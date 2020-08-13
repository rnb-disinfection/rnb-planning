sudo cp /opt/gcc-6.5/lib/libstdc++.so.6.0.22 /usr/lib/i386-linux-gnu
sudo rm -f /usr/lib/i386-linux-gnu/libstdc++.so.6
sudo ln -s /usr/lib/i386-linux-gnu/libstdc++.so.6.0.22 /usr/lib/i386-linux-gnu/libstdc++.so.6
