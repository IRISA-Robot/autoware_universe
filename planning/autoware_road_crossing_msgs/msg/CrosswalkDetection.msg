std_msgs/Header header
bool detected
float32 confidence
# image-space bbox crosswalk terdeteksi (kosong kalau tidak ada)
geometry_msgs/Polygon[] boxes
# zebra midline endpoints image-space [x1,y1,x2,y2], normalized 0..1 (opsional, 0 kalau tak ada)
float32[4] zebra_midline
