import face_recognition as fr
import numpy as np

# Load the png files to numpy arrays
stop_sign_image = fr.load_image_file('stop sign.png')

# Get encodings for face
stop_sign_encoding = fr.face_encodings(stop_sign_image)[0]

known_signs = [stop_sign_encoding]
singns_dict = {0:"Stop!"}

def detect_face(image):
    try:
        unknown_encoding = fr.face_encodings(image)[0]
    except IndexError:
        return 'Was not able to extract sign from unknown image'


    results = fr.face_distance(known_faces, unknown_encoding)
    results = np.argmin(results)
    results = faces_dict[results]
    return results


print detect_face(fr.load_image_file('unknown.png'))
