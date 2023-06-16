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

echo_and_run cd "/home/alex/ant_ws/src/ant_walking_rl/ant_model_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/alex/ant_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/alex/ant_ws/install/lib/python3/dist-packages:/home/alex/ant_ws/build/ant_model_control/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/alex/ant_ws/build/ant_model_control" \
    "/usr/bin/python3" \
    "/home/alex/ant_ws/src/ant_walking_rl/ant_model_control/setup.py" \
    egg_info --egg-base /home/alex/ant_ws/build/ant_model_control \
    build --build-base "/home/alex/ant_ws/build/ant_model_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/alex/ant_ws/install" --install-scripts="/home/alex/ant_ws/install/bin"
