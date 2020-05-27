# anymal-rbprm
File database for anymal robot using the hpp-rbprm framework

# Installation instruction

    mkdir build ; cd build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DPYTHON_EXECUTABLE=$(which python{X}) -DCMAKE_INSTALL_PREFIX={INSTALL_PATH} ..
    make install

Replace {X} with your Python version (2 or 3) and {INSTALL_PATH} with the desired path. 
