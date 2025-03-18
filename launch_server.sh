# Example of better error handling
if python3 robot_camera.py; then
  cd server
  gunicorn -w 12 -b 0.0.0.0:5000 -k gevent --timeout 0 --worker-connections 2 'monitor:app'
else
  echo "Camera initialization failed. Not starting web server."
  exit 1
fi