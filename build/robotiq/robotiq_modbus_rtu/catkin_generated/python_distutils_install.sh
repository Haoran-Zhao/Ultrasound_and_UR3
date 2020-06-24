#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/haoran/UR_ws/src/robotiq/robotiq_modbus_rtu"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/haoran/UR_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/haoran/UR_ws/install/lib/python2.7/dist-packages:/home/haoran/UR_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/haoran/UR_ws/build" \
    "/usr/bin/python2" \
    "/home/haoran/UR_ws/src/robotiq/robotiq_modbus_rtu/setup.py" \
     \
    build --build-base "/home/haoran/UR_ws/build/robotiq/robotiq_modbus_rtu" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/haoran/UR_ws/install" --install-scripts="/home/haoran/UR_ws/install/bin"
