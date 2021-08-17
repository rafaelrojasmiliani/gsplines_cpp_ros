
@ECHO OFF

docker build -t "gsplines_cpp_ros" ^
    --build-arg myuser="%USERNAME%" ^
    --build-arg myuid=11011 ^
    --build-arg mygroup="%USERNAME%" ^
    --build-arg mygid=11011 ^
    --no-cache -f ./image.dockerfile .
