# BluHubChip

1) Install WICED Development Studio:
https://www.cypress.com/documentation/development-kitsboards/cyw920719q40evb-01-evaluation-kit
2) Select Platform: `20719-B1_Bluetooth`

## Import (Git Cloning) Instructions
1) Open WICED Dev. Studio and select platform `20719-B1_Bluetooth`
2) Right click `20719-B1_Bluetooth` under Project Explorer and then click `Import...`
3) Select `Projects from Git` under the Git directory then `Next`
4) Select `Clone URI` and then `Next`
5) Enter the URI for the BluHubChip, port 443 if HTTPS, user credentials, and then `Next`
6) Select all of the branches then `Next`
7) Change destination directory to `WICED-Studio-6.2\20719-B1_Bluetooth\apps\` do NOT use the default install location, doing so will result in build failure.
8) Select `Import as general project` and then `Next`
9) `Finish`

Note:
The newly imported project will not be listed under Project Explorer if the `20719-B1_Bluetooth` Working Set is selected. Deselecting the Working Sets will then display the imported project.

## Build Instructions 
1)  Make sure that the correct platform is selected in WICED Studio under `WICED Platform` -> `20719-B1_Bluetooth`
2) Under the `Make Target` menu, right click on `20719-B1_Bluetooth` create a new build target.
3) Leave all the defaults and enter a target name of `BluHubChip-CYW920719Q40EVB_01 download`. 
- Note that `download` can be replaced with `build` if the binaries aren't to be pushed to the board. The `Makefile` under the `20719-B1_Bluetooth` project also contains more information on the build options/flags
4) Click `OK` to finish the build target. If `download` is included in the build commands then the plug is to be plugged in before building the target.

## Device Discovery
If the studio cannot find the board, try using recovery mode. To put the board into recovery mode, use the tactile buttons on the developement board and do: press down recover > press down reset > release reset > release recover


## Using WICED Client control for a basic one device AV Source connection
1) Connect the board and identify what comm port is being used for the UART connection. The board will always extablish 2 serial port connections, a UART and PUART. UART is generally the lower of the two ports if it is not labeled
2) Make sure that the project is built and downloaded onto the board
3) Open client control under `20719-B1_Bluetooth/apps/host/client_control/Windows/ClientControl.exe` (from within WICED studio) or `WICED-Studio-6.2\common\client_control\Windows\ClientControl.exe` (from file explorer). Note that directories are different inside WICED studio because they use a lot of sybolic links
4) At the top-left of client control, use the drop-down menu to select the proper UART serial port.
5) Select 3000000 as the baud rate
6) Click `Open port`
7) Put a Bluetooth sound device of your choice into pairing mode and click `Start` under `BR/EDR Discovery`
8) The name and address of your sound device should appear in the drop-down menu under `BR/EDR Discovery`, select it and `Stop` discovery
9) Go to the `AV Source` tab and click `Connect`
10) Under `Media`, you may choose a music file or a frequency to play over the sound device. The application only supports `.wav` files
11) Click `Start Streaming` and the frequency or `.wav` file should start playing
Notes:
Putting your sound device in pairing mode should only be necessary on the first connection. If you have difficulty connecting to the same audio device more than once, try selecting it and clicking `unbond` to make the board forget the device and pair again.