FROM debian:testing

# Set default shell during Docker image build to bash
SHELL ["/bin/bash", "-c"]

LABEL org.opencontainers.image.authors="delling@silpion.de"

# Set non-interactive frontend for apt-get to skip any user confirmations
ENV DEBIAN_FRONTEND=noninteractive

# Set the default zephyr version to use
ARG ZEPHYR_VER=v3.4.0

# Install base packages
RUN apt-get -y update && \
	apt-get -y upgrade && \
	apt-get install --no-install-recommends -y \
		build-essential \
		ca-certificates \
		cargo \
		ccache \
		cmake \
		device-tree-compiler \
		dfu-util \
		diffstat \
		dos2unix \
		file \
		flex \
		g++ \
		gawk \
		gcc \
		gcovr \
		git \
		git-core \
		gnupg \
		gperf \
		libcairo2-dev \
		libncurses6 \
		libgit2-dev \
		libgirepository1.0-dev \
		libmagic1 \
		libusb-1.0-0-dev udev \
		libxrandr2 \
		locales \
		make \
		ninja-build \
		openssh-client \
		pkg-config \
		python3-dev \
		python3-full \
		python3-pip \
		python3-ply \
		python3-setuptools \
		python3-wheel \
		python-is-python3 \
		sudo \
		vim \
		wget \
		xz-utils

# Install multi-lib gcc (x86 only)
RUN if [ "${HOSTTYPE}" = "x86_64" ]; then \
	apt-get install --no-install-recommends -y \
		gcc-multilib \
		g++-multilib \
	; fi

# Gcc 12 does not work yet
RUN wget https://developer.arm.com/-/media/Files/downloads/gnu/11.3.rel1/binrel/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi.tar.xz && \
	tar xf arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi.tar.xz -C /opt && \
	rm arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi.tar.xz

# Initialise system locale
ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

# Clean up stale packages
RUN apt-get clean -y && \
	apt-get autoremove --purge -y && \
	rm -rf /var/lib/apt/lists/*

# Create 'user' account
RUN groupadd -g 1000 -o user && \
	mkdir -p /build && \
	useradd -u 1000 -m -g user -d /build -G plugdev user && \
	chown user:user /build && \
	echo 'user ALL = NOPASSWD: ALL' > /etc/sudoers.d/user && \
	chmod 0440 /etc/sudoers.d/user && \
	rm /usr/lib/python3.11/EXTERNALLY-MANAGED

# Install Python dependencies not available as debian packages (--break-system-packages)
RUN python3 -m pip install pygobject pyelftools west

# Prepare
WORKDIR build
USER user
ENV ZEPHYR_BASE=/build/zephyrproject/zephyr
ENV ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
ENV GNUARMEMB_TOOLCHAIN_PATH=/opt/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi

# Install Micropython
RUN git clone --depth 1 --branch master https://github.com/micropython/micropython

# Install Zephyr
RUN west init zephyrproject -m https://github.com/zephyrproject-rtos/zephyr --mr ${ZEPHYR_VER} && \
	cd zephyrproject && \
	west update

RUN west build -b nrf52840dk_nrf52840 /build/micropython/ports/zephyr

# Build: $ docker build -t mpbuild . [--build-arg ZEPHYR_VER=main]
# Use: $ docker run -it mpbuild
