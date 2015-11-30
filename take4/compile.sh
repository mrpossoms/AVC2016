PKG_CONFIG_PATH=/usr/lib/pkgconfig:${PKG_CONFIG_PATH}
export PKG_CONFIG_PATH
gcc $(pkg-config --cflags --libs opencv) -lm -o objectDetection main.c
