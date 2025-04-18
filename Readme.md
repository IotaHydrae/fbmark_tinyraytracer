# TinyRayTracer Linux Framebuffer Benchmark

The original project is: [https://github.com/ssloy/tinyraytracer](https://github.com/ssloy/tinyraytracer)

This benchmark is designed for measuring framebuffer
rendering performance on embedded Linux dev boards.

## Build & Run

### Run it on Luckfox Pico

For example, you can cross-compile for `Luckfox Pico`. SDK is required.

```shell
mkdir build && cd build

export CC=/home/developer/luckfox/pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc
export CXX=/home/developer/luckfox/pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-g++

cmake .. -G Ninja

ninja && adb push tinyraytracer /root
adb shell "chmod +x /root/tinyraytracer && /root/tinyraytracer"
```

A 1.8" 128x160 ST7735R TFT display is connected via SPI.
After starting the benchmark, the raytracing demo scene will appear on the screen.

Once the benchmark is complete, the results will be printed to the console.

```shell
Starting Benchmark...
Rendered 100 frames in 10.1256 seconds.
Average FPS: 9.876
```

That's it — happy hacking! 🚀

### Run it on Luckfox Lyra

```shell
mkdir build && cd build

export CC=${HOME}/luckfox/lyra/prebuilts/gcc/linux-x86/arm/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-gcc
export CXX=${HOME}/luckfox/lyra/prebuilts/gcc/linux-x86/arm/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-g++

cmake .. -G Ninja
ninja && adb push tinyraytracer /root
```

It’s a bit different from the Luckfox Pico. After setting CC and CXX to the SDK toolchain and building the project successfully, you might also need to copy libgomp.so.1 to the device’s /lib directory.

```shell
adb push /home/developer/luckfox/lyra/prebuilts/gcc/linux-x86/arm/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf/arm-none-linux-gnueabihf/lib/libgomp.so.1.0.0 /root
```

at device side, link it to `libgomp.so.1`:
```shell
cd /lib
mv /root/libgomp.so.1.0.0 .
ln -sf libgomp.so.1.0.0 libgomp.so.1
```

Then you can run the benchmark without any problem.
```shell
chmod +x /root/tinyraytracer && /root/tinyraytracer
```
