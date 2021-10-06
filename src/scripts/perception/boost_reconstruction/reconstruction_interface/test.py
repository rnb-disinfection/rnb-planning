import reconstruction_interface_py as rci

#rci.greet(10, 12.2, 123.12, 123.4344, 5)
rci.getReconstruction("../dataset/color", "../dataset/depth", "../dataset/intrinsic.json")
