#!/bin/bash

#Read the calibration params file
params_file='/home/miguel/catkin_ws/src/uw-calibration-pinax/input_data/calibration_params.txt'
params[0]='start'
cc=0

while IFS='' read -r line || [[ -n "$line" ]]; do
    echo "Text read from file: $line"
        let c=c+1
        params[$c]="$(cut -d' ' -f2 <<<$line)"
done < "$params_file"



/bin/bash -c "rosrun defraction_map_finder stereo_defraction_map_finder -i /home/miguel/catkin_ws/src/uw-calibration-pinax/output_data/calibration_files -o /home/miguel/catkin_ws/src/uw-calibration-pinax/output_data/pinax_maps --refraction-d0 ${params[8]} --refraction-d0-offset ${params[9]} --refraction-d1 ${params[10]} --refraction-n-glass ${params[11]} --refraction-n-water ${params[12]} --rectified-f ${params[13]} --rectified-width ${params[14]} --rectified-height ${params[15]} --rectified-cx ${params[16]} --rectified-cy ${params[17]}"
chmod a+rw /home/miguel/catkin_ws/src/uw-calibration-pinax/output_data*


#rosrun defraction_map_finder stereo_defraction_map_finder -i /home/miguel/catkin_ws/src/uw-calibration-pinax/output_data/calibration_files -o /home/miguel/catkin_ws/src/uw-calibration-pinax/output_data/pinax_maps --refraction-d0 0.035 --refraction-d0-offset 0.00165 --refraction-d1 0.008 --refraction-n-glass 1.7751  --refraction-n-water 1.34 --rectified-f 2774 --rectified-width 1920 --rectified-height 1216 .--rectified-cx 960 --rectified-cy 608"

#rosrun defraction_map_finder stereo_defraction_map_finder -i /home/miguel/catkin_ws/src/uw-calibration-pinax/output_data/calibration_files -o /home/miguel/catkin_ws/src/uw-calibration-pinax/output_data/pinax_maps --refraction-d0 0.035 --refraction-d0-offset 0.0165 --refraction-d1 0.008 --refraction-n-glass 1.7751  --refraction-n-water 1.34 --rectified-f 2704.197396 --rectified-width 1920 --rectified-height 1216 --rectified-cx 969.445747 --rectified-cy 602.449646 --show-masks true


#rosrun defraction_map_finder stereo_defraction_map_finder -i /home/miguel/catkin_ws/src/uw-calibration-pinax/output_data/calibration_files -o /home/miguel/catkin_ws/src/uw-calibration-pinax/output_data/pinax_maps --refraction-d0 0.035 --refraction-d0-offset 0.0165 --refraction-d1 0.008 --refraction-n-glass 1.7751  --refraction-n-water 1.34 --rectified-f 2704.0572102790347 --rectified-width 1920 --rectified-height 1216 --rectified-cx 968.9917373657227 --rectified-cy 602.4532623291016 --show-masks true

