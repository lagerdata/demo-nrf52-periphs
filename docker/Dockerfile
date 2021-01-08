FROM lagerdata/devenv-cortexm-minimal

RUN apt-get update \
	&& apt-get install -y wget \
	&& wget "https://ziglang.org/builds/zig-linux-x86_64-0.8.0-dev.847+148c887ac.tar.xz" \
	&& tar -xf zig-linux-x86_64-0.8.0-dev.847+148c887ac.tar.xz \
	&& rm zig-linux-x86_64-0.8.0-dev.847+148c887ac.tar.xz \
	&& apt-get remove -y wget \
	&& apt-get autoremove -y \
	&& rm -rf /var/lib/apt/lists/*

ENV PATH="/app/zig-linux-x86_64-0.8.0-dev.847+148c887ac:${PATH}"