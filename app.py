from flask import Flask, render_template, request, jsonify
from pyngrok import ngrok
import os
import threading
from traffic_control import start_traffic_control, set_video_paths

app = Flask(__name__,template_folder="/content/drive/MyDrive/Traffic Ease/templates")

# Set video paths through Flask route
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/upload', methods=['POST'])
def upload_files():
    uploaded_files = request.files.getlist('videos')
    video_paths = []
    upload_dir = 'uploads'

# Create the directory if it doesn't exist
    if not os.path.exists(upload_dir):
       os.makedirs(upload_dir)
    # Save uploaded videos
    for file in uploaded_files:
        path = os.path.join('uploads', file.filename)
        file.save(path)
        video_paths.append(path)

    set_video_paths(video_paths)  # Set video paths for traffic control
    return jsonify({"message": "Videos uploaded successfully.", "video_paths": video_paths})

@app.route('/start', methods=['POST'])
def start_traffic_control_route():
    # Start the traffic control in a separate thread
    t = threading.Thread(target=start_traffic_control, daemon=True)
    t.start()
    return jsonify({"message": "Traffic control started."})

if __name__ == "__main__":
    # Expose Flask app on a public URL
    #public_url = ngrok.connect(5000)
    public_url = ngrok.connect(5000)
    print(f" * Ngrok tunnel \"{public_url}\" -> \"http://127.0.0.1:5000\"")  # Use a different port
    app.run(debug=True,use_reloader=False)
