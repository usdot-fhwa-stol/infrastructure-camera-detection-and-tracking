
FOLDER="/home/$USER/fcp_ws/other/MUST UDP Data_Week2_v1.0/uw_processed_9-18"
cd "${FOLDER}"
mkdir -p video_frames
# VIDEO_FOOTER=_${FOLDER}_detections.mp4
VIDEO_FOOTER=_output.avi
ffmpeg -y -i 2024-09-12_16-18-40${VIDEO_FOOTER} -vf "select=gte(n\,305)" -vframes 1 video_frames/2024-09-12_16-18-40_vehicle.png
ffmpeg -y -i 2024-09-12_16-22-03${VIDEO_FOOTER} -vf "select=gte(n\,850)" -vframes 1 video_frames/2024-09-12_16-22-03_vehicle.png
ffmpeg -y -i 2024-09-12_16-24-56${VIDEO_FOOTER} -vf "select=gte(n\,1100)" -vframes 1 video_frames/2024-09-12_16-24-56_vehicle.png
ffmpeg -y -i 2024-09-12_16-28-14${VIDEO_FOOTER} -vf "select=gte(n\,175)" -vframes 1 video_frames/2024-09-12_16-28-14_vehicle.png
ffmpeg -y -i 2024-09-12_16-30-37${VIDEO_FOOTER} -vf "select=gte(n\,1275)" -vframes 1 video_frames/2024-09-12_16-30-37_vehicle.png
ffmpeg -y -i 2024-09-12_16-32-47${VIDEO_FOOTER} -vf "select=gte(n\,65)" -vframes 1 video_frames/2024-09-12_16-32-47_vehicle.png
ffmpeg -y -i 2024-09-12_16-36-18${VIDEO_FOOTER} -vf "select=gte(n\,90)" -vframes 1 video_frames/2024-09-12_16-36-18_vehicle.png
ffmpeg -y -i 2024-09-12_16-37-50${VIDEO_FOOTER} -vf "select=gte(n\,85)" -vframes 1 video_frames/2024-09-12_16-37-50_vehicle.png
ffmpeg -y -i 2024-09-12_16-42-05${VIDEO_FOOTER} -vf "select=gte(n\,200)" -vframes 1 video_frames/2024-09-12_16-42-05_vehicle.png
ffmpeg -y -i 2024-09-12_16-45-00${VIDEO_FOOTER} -vf "select=gte(n\,1015)" -vframes 1 video_frames/2024-09-12_16-45-00_vehicle.png
ffmpeg -y -i 2024-09-12_16-50-45${VIDEO_FOOTER} -vf "select=gte(n\,145)" -vframes 1 video_frames/2024-09-12_16-50-45_vehicle.png
ffmpeg -y -i 2024-09-12_16-53-03${VIDEO_FOOTER} -vf "select=gte(n\,125)" -vframes 1 video_frames/2024-09-12_16-53-03_vehicle.png
