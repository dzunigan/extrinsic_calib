# Extrinsic calibration of a set of range cameras

A C++ implementation of [\[1\]][1], with modifications as described in [\[2\]][2].

## Dependencies

 - MRPT [1.4]
 - PCL [1.7]
 - Eigen [3.0]
 - Boost (filesystem; system) [1.63.0]

## Usage notes

Run the calibration process with:
```
./build/extrinsic_calib ./data/config.ini
```
where the file _config.ini_ holds all required information. For an example of input rawlog file, download: [rgbd_calib.rawlog](https://drive.google.com/file/d/1mTN7hovdiMku-l4dXY1NeBJ2Guek3If6/view?usp=sharing)

## References

Some code has been adapted from [EduFdez](https://github.com/EduFdez/mrpt)

\[1\] E. Fernandez-Moral et. al, Extrinsic calibration of a set of range cameras in 5 seconds without pattern, IROS 2014

\[2\] D. Zúñiga-Noël, R. Gómez Ojeda, F. A. Moreno and J. González Jiménez, Calibración Extrínseca de un Conjunto de Cámaras RGB-D sobre un Robot Móvil, JJAA 2017

[1]: doc/rgbd_calib.pdf
[2]: doc/jjaa17.pdf
