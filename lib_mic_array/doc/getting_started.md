# Getting Started

There are three models for how the PDM mic array component can be included in 
your application. The details of how to allocate, initialize and start the mic
array will depend on the chosen model.

These are:

* Vanilla Model - This is the simplest way to include the mic array. It is 
usually sufficient but offers relatively little flexibility with respect to 
configuration and run-time control. Using this model (mostly) means modifying an
application's build scripts.

* Prefab Model - This model involves a little more work from the application
developer, including writing a couple C++ wrapper functions, but gives the 
application access to any of the defined prefab mic array components.

* General Model - This model is necessary if an application wishes to use a 
customized mic array component.

The vanilla and prefab models for integrating the mic array into your 
application will be discussed in more detail below. The general model may 
customizing or extending the classes in `lib_mic_array` and is beyond the scope
of this introduction. Whichever model is chosen, the first step is to _identify 
your hardware resources_.


## Identify Resources

The key hardware resources to be identified are the _ports_ and _clock blocks_ 
that will be used by the mic array component.  The ports correspond to the 
physical pins on which clocks and sample data will be signaled.  Clock blocks 
are a type of hardware resource which can be attached to ports to coordinate the
presentation and capture of signals on the pins.

### Clock Blocks

While clock blocks may be more abstract than ports, their implications for this
library are actually simpler. First, the mic array component will need a way of 
taking the audio master clock and dividing it to produce a PDM sample clock.
This can be accomplished with a clock block. This will be the clock block which
the API documentation refers to as "Clock A".

Second, if (and only if) the PDM microphones are being used in a Dual Data Rate 
(DDR) configuration a second clock block will be required. In a DDR 
configuration 2 microphones share a physical pin for output data, where one 
signals on the rising edge of the PDM clock and the other signals on the falling 
edge. The second clock block required in a DDR configuration is referred to as 
"Clock B" in the API documentation.

Each tile on an xcore.ai device has 5 clock blocks available. In code, a clock
block is identified by its resource ID, which are given as the preprocessor
macros `XS1_CLKBLK_1` through `XS1_CLKBLK_5`. 

Unlike ports, which are tied to specific physical pins, clock blocks are 
fungible. Your application is free to use any clock block that has not already
been allocated for another purpose. The vanilla component model defaults to 
using `XS1_CLKBLK_1` and `XS1_CLKBLK_2`.

### Ports

Three ports are needed for the mic array component. As mentioned above, ports 
are physically tied to specific device pins, and so the correct ports must be
identified for correct behavior.

Note that while ports are physically tied to specific pins, this is _not_ a 
1-to-1 mapping. Each port has a port width (measured in bits) which is the 
number of pins which comprise the port. Further, the pin mappings for different
ports _overlap_, with a single pin potentially belonging to multiple ports. When
identifying the needed ports, take care that both the pin map (see the 
documentation for your xcore.ai package) and port width are correct.

The first port needed is a 1-bit port on which the audio master clock is 
received. In the code and various places in the documentation, you may see this
referred to as `p_mclk`.

The second port needed is a 1-bit port on which the PDM clock will be signaled 
to the PDM mics. This port is often referred to as `p_pdm_clk`.

The third port is that on which the PDM data is received. For 1 microphone 
operating in an SDR configuration or 2 microphones in a DDR configuration, this
will be a 1-bit port. For 4 microphones in SDR or 8 in DDR, this will be a 4-bit
port. In either case, this port is referred to as `p_pdm_mics`.

XCore applications are typically compiled with an "XN" file (with a ".xn" file 
extension). An XN file is an XML document which describes some information about
the device package as well as some other helpful board-related information. The 
identification of your ports may have already been done for you in your XN file.
Here is a snippet from an XN file:

    ...
    <Tile Number="1" Reference="tile[1]">
      <!-- MIC related ports -->
      <Port Location="XS1_PORT_1G"  Name="PORT_PDM_CLK"/>
      <Port Location="XS1_PORT_1F"  Name="PORT_PDM_DATA"/>
      <!-- Audio ports -->
      <Port Location="XS1_PORT_1D"  Name="PORT_MCLK_IN_OUT"/>
      <Port Location="XS1_PORT_1C"  Name="PORT_I2S_BCLK"/>
      <Port Location="XS1_PORT_1B"  Name="PORT_I2S_LRCLK"/>
      <!-- Used for looping back clocks -->
      <Port Location="XS1_PORT_1N"  Name="PORT_NOT_IN_PACKAGE_1"/>
    </Tile>
    ...

The first 3 ports listed, `PORT_PDM_CLK`, `PORT_PDM_DATA` and `PORT_MCLK_IN_OUT`
are respectively `p_pdm_clk`, `p_pdm_mics` and `p_mclk`. The value in the 
"Location" attribute (e.g. `XS1_PORT_1G`) is the port name as you will find it
in your package documentation. 

In this case, either `PORT_PDM_CLK` or `XS1_PORT_1G` can be used in code to 
identify this port.

### Declaring Resources

Once the ports and clock blocks to be used have been indentified, these 
resources can be represented in code using a `pdm_rx_resources_t` struct. The
following is an example of declaring resources in a DDR configuration. See 
@ref `pdm_rx_resources_t`, @ref `PDM_RX_RESOURCES_SDR()` and @ref 
`PDM_RX_RESOURCES_DDR()` for more details.

    pdm_rx_resources_t pdm_res = PDM_RX_RESOURCES_DDR(
                                    PORT_MCLK_IN_OUT,
                                    PORT_PDM_CLK,
                                    PORT_PDM_DATA,
                                    XS1_CLKBLK_1,
                                    XS1_CLKBLK_2);

Note that this is not necessary when using the vanilla model, as it is done for 
you.

### Other Resources

In addition to ports and clock blocks, there are also several other hardware 
resource types used by `lib_mic_array` which are worth considering. Running out
of any of these will preclude the mic array from running correctly (if at all)

* Threads - At least one hardware thread is required to run the mic array 
component. A second thread may also be used for modestly reduced MIPS 
consumption.

* Computation - The mic array component will require a fixed number of MIPS 
(millions of instructions per second) to perform the required processing. The 
exact amount will depend on the configuration used.

* Memory - The mic array requires a modest amount of memory for code and data. 
(see @todo).

* Chanends - At least 4 chanends must be available for signaling between 
threads/sub-components.


## Vanilla Model

Mic array configuration with the vanilla model is achieve mostly through the
application's build system configuration.

In the `/etc/vanilla` directory of this repository are a source and header file
which are not compiled with (or on the include path) of the library. Configuring
the mic array in vanilla mode means adding those files to your _application_'s
build (_not_ the library target), and defining several compiler flags which tell
it how to behave.

### Vanilla - CMake Macro

To simplify this further, a CMake macro called `mic_array_vanilla_add()` has 
been included with the build system.

`mic_array_vanilla_add()` takes several arguments:

* `TARGET_NAME` - The name of the CMake application target that the vanilla mode
source should be added to.
* `MCLK_FREQ` - The frequency of the master audio clock, in Hz.
* `PDM_FREQ` - The desired frequency of the PDM clock, in Hz.
* `MIC_COUNT` - The number of microphone channels to be captured.
* `SAMPLES_PER_FRAME` - The size of the audio frames produced by the mic array 
component (frames will be 2 dimensional arrays with shape 
`(MIC_COUNT, SAMPLES_PER_FRAME)`).

### Vanilla - Optional Configuration

Though not exposed by the `mic_array_vanilla_add()` macro, several additional
configuration options are available when using the vanilla model. These are all
configured by adding defines to the application target.

### Vanilla - Initializing and Starting

Once the configuration options have been chosen, initializing and starting the 
mic array at run-time is achieved easily. Two function calls are necessary, both
can be included through `mic_array_vanilla.h`.

First, during application initialization, the function `ma_vanilla_init()`, 
which takes no arguments, must be called. This will configure the hardware
resources and install the PDM rx service as an ISR, but will not actually start
any threads or PDM capture.

Then, once initialization is complete, to begin PDM capture and processing, the
vanilla thread entry point, `ma_vanilla_task()` is called. `ma_vanilla_task()`
takes a single argument which is the chanend that will be used to transmit audio
frames to subsequent stages of the processing pipline. Usually the call to
`ma_vanilla_task()` will be placed directly in a `par {...}` block along with
other threads do be started on the tile.

(Note that these functions must be called from the tile which will host the 
decimator thread)

## Prefab Model

The `lib_mic_array` library has a C++ namespace `mic_array::prefab` which 
contains class templates for typical mic array setups using common 
sub-components. The intention is to hide most of the complexity (and unneeded 
flexibility) from the application author, so they can focus only on pieces they
care about.

(Note, at the time of this writing, only one prefab class template, 
`mic_array::prefab::BasicMicArray` has been defined.)

To configure the mic array using a prefab, you will need to add a C++ source
file to your application. NB: This will end up looking a lot like the contents
of `mic_array_vanilla.cpp` when you are through.

### Prefab - Declare Resources

The example in this section will use `2` microphones in a DDR configuration with
DC offset elimination enabled, and using 128-sample frames. The resource IDs 
used may differ than those required for your application.

`pdm_res` will be used to identify the ports and clocks which will be configured
for PDM capture.

    #include "mic_array/cpp/Prefab.hpp"
    ...
    #define MIC_COUNT    2    // 2 mics
    #define DCOE_ENABLE  true // DCOE on
    #define FRAME_SIZE   128  // 128 samples per frame
    ...
    pdm_rx_resources_t pdm_res = PDM_RX_RESOURCES_DDR(
                                    PORT_MCLK_IN_OUT,
                                    PORT_PDM_CLK,
                                    PORT_PDM_DATA,
                                    MIC_ARRAY_CLK1,
                                    MIC_ARRAY_CLK2);

### Prefab - Allocate MicArray

The C++ class template `mic_array::MicArray` is central to the mic array 
component in this library. The class templates defined in the 
`mic_array::prefab` namespace each derive from `mic_array::MicArray`.

Define and allocate the specific implementation of `MicArray` to be used.

    // Using the full name of the class could get cumbersome. Let's give it an 
    // alias.
    using TMicArray = mic_array::prefab::BasicMicArray<
                          MIC_COUNT, FRAME_SIZE, DCOE_ENABLED>
    // Allocate mic array
    TMicArray mics = TMicArray();

Now the mic array component has been defined and allocated. Because class 
templates were used, the `mics` object is self-contained, without the need of
external data buffers. Additionally, class templates will ultimately allow 
unused features to be optimized out at build time. For example, if DCOE is
disabled, it will be optimized out so that at run-time there won't even be a 
check to see whether it's enabled.

### Prefab - Init and Start Functions

You'll now need to implement a couple functions in your C++ file. In most cases 
these functions will need to be callable from C or XC, and so they should not be 
static, and they should be decorated with `extern "C"` (or the `MA_C_API`)
preprocessor macro provided by the library.

First, a function which initializes the `MicArray` object and configures the 
port and clock block resources.  The documentation for 
`mic_array::prefab::BasicMicArray` will indicate any parts of the `MicArray`
object that need to be initialized.

    #define MCLK_FREQ   24576000
    #define PDM_FREQ    3072000
    ...
    MA_C_API
    void app_init() {
      // Configure clocks and ports
      const unsigned mclk_div = mic_array_mclk_divider(MCLK_FREQ, PDM_FREQ);
      mic_array_resources_configure(&pdm_res, mclk_div);

      // Initialize the PDM rx service
      mics.PdmRx.Init(pdm_res.p_pdm_mics);
    }

`app_init()` can be called from an XC `main()` during initialization.

For this example we'll assume we want to run the PDM rx service as an ISR. We'll
start the PDM clock, install the ISR and enter the decimator thread.

    MA_C_API
    void app_mic_array_task(chanend_t c_audio_frames) {
      mics.SetOutputChannel(c_audio_frames);

      // Start the PDM clock
      mic_array_pdm_clock_start(&pdm_res);

      mics.InstallPdmRxISR();
      mics.UnmaskPdmRxISR();

      mics.ThreadEntry();

Now a call to `app_mic_array_task()` with the channel to send frames on can be
placed inside a `par {...}` block to spawn the thread.