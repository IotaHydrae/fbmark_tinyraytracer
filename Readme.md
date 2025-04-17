# TinyRayTracer Linux Framebuffer Benchmark

The original project is: [https://github.com/ssloy/tinyraytracer](https://github.com/ssloy/tinyraytracer)

This benchmark is designed for measuring framebuffer
rendering performance on embedded Linux dev boards.

## Build & Run

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
