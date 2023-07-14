export calibration_json_dir='C://Users//HP//Desktop//hzq//hanglok-robotics//nerf//processed_07_06_18_40//transforms.json'
export calibration_image_dir='C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_07_06_16_38'
export save_dir='C://Users//HP//Desktop//hzq//data4show//test_0714_1508'
export video_camera_path='C://Users//HP//Desktop//hzq//hanglok-robotics//nerf//processed_07_06_18_40//base_cam.json'

python C://Users//HP//Desktop//hzq//hanglok-robotics//collect_data4show.py --calibration_json_dir=$calibration_json_dir --calibration_image_dir=$calibration_image_dir --save_dir=$save_dir

# C://Users//HP//Desktop//hzq//Instant-NGP-for-RTX-3000-and-4000//instant-ngp.exe --scence $save_dir

python C://Users//HP//Desktop//hzq//Instant-NGP-for-RTX-3000-and-4000//scripts//run.py --video_camera_path $video_camera_path --video_n_seconds 5 --video_fps 60 --render_name test_0714

python C:\Users\HP\Desktop\hzq\Instant-NGP-for-RTX-3000-and-4000\scripts\run.py --scene 'C://Users//HP//Desktop//hzq//data4show//07_14_12_50' --video_camera_path 'C://Users//HP//Desktop//hzq//hanglok-robotics//nerf//processed_07_06_18_40//base_cam.json' --video_n_seconds 5 --video_fps 60 --render_name test_0714 
