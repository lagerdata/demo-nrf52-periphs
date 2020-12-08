FROM lagerdata/devenv-cortexm-minimal

RUN apt-get update \
	&& apt-get install -y wget \
	&& wget "https://ziglang.org/builds/zig-linux-x86_64-0.7.0+58241364c.tar.xz" \
	&& tar -xf zig-linux-x86_64-0.7.0+58241364c.tar.xz \
	&& rm zig-linux-x86_64-0.7.0+58241364c.tar.xz \
	&& apt-get remove -y wget \
	&& apt-get autoremove -y \
	&& rm -rf /var/lib/apt/lists/*

ENV PATH="/app/zig-linux-x86_64-0.7.0+58241364c:${PATH}"