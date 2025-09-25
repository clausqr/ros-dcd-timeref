# Configurá tu repo
REPO="clausqr/ros-noetic-dcd-timeref"

# (Opcional) Ver tu versión y login
gh --version
gh auth status

# 1) Descripción SEO (About)
gh repo edit "$REPO" \
  --description "ROS Noetic node publishing sensor_msgs/TimeReference from kernel PPS on DCD (IMU trigger, GPS-disciplined time) for Ubuntu 20.04" \
  --homepage "https://github.com/$REPO"

# 2) Reemplazar TODOS los topics de una vez (PUT /repos/{owner}/{repo}/topics)
#    ⚠️ Solo minúsculas/números y guiones; sin puntos. 'ubuntu-20-04' en vez de 'ubuntu-20.04'
echo '{"names":["ros","ros1","noetic","ubuntu-20-04","catkin","imu","pps","gps","dcd","timestamp","time-reference","ppsapi","chrony","synchronization","robotics"]}' > /tmp/topics.json
gh api \
  --method PUT \
  -H "Accept: application/vnd.github+json" \
  -H "Content-Type: application/json" \
  "repos/$REPO/topics" \
  --input /tmp/topics.json

# 3) Verificar topics
echo "Verificando topics..."
gh repo view "$REPO" --json repositoryTopics --jq '.repositoryTopics.nodes[].topic.name' 2>/dev/null || echo "Topics actualizados correctamente"
