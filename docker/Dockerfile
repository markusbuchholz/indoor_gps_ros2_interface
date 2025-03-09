FROM ubuntu:16.04

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

RUN apt-get update && \
    apt-get -y --quiet --no-install-recommends install \
    libgl1-mesa-glx \
    libegl1-mesa \
    libxext6 \
    libx11-6 \
    nano \
    libgtk2.0-0 \
    libcanberra-gtk-module \
    libglu1-mesa \
    libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /etc/udev/rules.d

RUN echo '#Marvelmind serial port rules\nKERNEL=="ttyACM0",GROUP="dialout",MODE="666"' > /etc/udev/rules.d/99-tty.rules

RUN adduser --disabled-password --gecos '' user && adduser user dialout

# copy libdashapi.so to /usr/local/lib and run ldconfig
COPY ../src/x86/libdashapi.so /usr/local/lib/
RUN ldconfig

WORKDIR /workspace

COPY ../src/x86/dashboard_x86 /workspace/

RUN chmod 0777 /workspace/dashboard_x86



