# RH3 - Color detector publisher
## Stop Watchtower
`dts devel watchtower stop -H maxicar.local`
`
## Build
`dts devel build -f --arch arm32v7 -H maxicar.local`

## Run:
`docker -H maxicar.local run -it --rm --privileged --net host duckietown/dt-duckiebot-colordetector-publisher:v1-arm32v7`

## Show GUI
`dts start_gui_tools maxicar --base_image duckietown/dt-core:daffy-amd64`

`rqt_image_view`