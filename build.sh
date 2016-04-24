clang src/*/*.cpp -c -static -std=c++11 -D LIBNIFALCON_USE_LIBUSB
clang++ c_api/main.cpp -I include -std=c++11 -stdlib=libstdc++ -mmacosx-version-min=10.6 -lusb-1.0 -lpthread *.o -shared -o pyfalcon.so
