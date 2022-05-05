# PokiRobotSoftMbed
This is a FEUO (For Emergency Use Only) Mbed repository to get a basic working PokiRobot soft.

Consider this as a "sandbox example" to use STM32 modules through MBED on the robot.

This project use Platformio (PIO) as building system.



## Problem with Python 3.10

If you have problem compiling or using PlatformIO with Python 3.10, you have two choice :

* Either downgrade to Python3.9
* Or fix Python3.10 : In every Python file that PIO report as a bug, replace `collections` import by `collections.abc` [as explain here](https://stackoverflow.com/a/72032154).



## Mbedignore

To accelerate compilation of MBED through PIO, you can use a `.mbedignore` file.

There already is one in the root of this project. To add Mbedignore support, simply [apply this Pull Request](https://github.com/platformio/builder-framework-mbed/pull/26) to your current PIO installation. If you don't know how, follow theses steps manually :

* Download [pio_mbed_adapter.py](https://raw.githubusercontent.com/platformio/builder-framework-mbed/f4e342ae23f16be730975b2a5b8544779cee1ddc/pio_mbed_adapter.py)
* Download [pio_mbedignore_loader.py](https://raw.githubusercontent.com/platformio/builder-framework-mbed/f4e342ae23f16be730975b2a5b8544779cee1ddc/pio_mbedignore_loader.py)
* Go to `C:\Users\<YOUR_NAME>\.platformio\packages\framework-mbed\platformio`
* Copy the two files you've just download (replace the original `pio_mbed_adapter.py` script)

Now you can clean the project, and rebuild it :

```
cd <your_project_root_folder_location>
pio run -t clean
pio run
```

