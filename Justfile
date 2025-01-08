network:
    sudo sysctl -w net.core.rmem_max=2147483647
    sudo sysctl -w net.core.wmem_max=2147483647
    sudo sysctl -w net.ipv4.ipfrag_time=3
    sudo sysctl -w net.ipv4.ipfrag_high_thresh=1342177280

start CONTAINER: network
    env DISPLAY=${DISPLAY:-0} docker compose run --rm {{CONTAINER}}

build_jazzy_zenoh:
  @echo "Building jazzy image with rmw_zenoh_cpp"
  docker build --progress=plain -f Dockerfile.jazzy.zenoh -t jazzy:zenoh .

build_ouster:
  @echo "Building ouster docker image"
  docker build --progress=plain -f Dockerfile.jazzy.ouster -t ouster:zenoh .

build: build_jazzy_zenoh
  @echo "Building all docker images"
