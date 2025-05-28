import cv2
import torch
import os
from datetime import datetime, timedelta
import numpy as np
from cv_stereo_ir.stereo_capture import StereoVideoProcessor
from cv_stereo_ir.models import EnhancedClassifier
from cv_stereo_ir.image_processing import compute_depth_map_advanced, enhanced_retinex

def compute_frame_similarity(frame1, frame2):
    """
    Calcola la similarità tra due frame usando MSE (Mean Squared Error).
    Returns: True se i frame sono simili, False altrimenti
    """
    if frame1 is None or frame2 is None:
        return False

    # Ridimensiona i frame per velocizzare il confronto
    size = (160, 120)  # Risoluzione ridotta per il confronto
    f1 = cv2.resize(frame1, size)
    f2 = cv2.resize(frame2, size)

    # Calcola MSE
    mse = np.mean((f1 - f2) ** 2)
    # Soglia di similarità (da regolare in base alle tue necessità)
    return mse < 1000  # Soglia da calibrare

def check_cameras():
    """
    Verifica la disponibilità delle telecamere.
    Returns: (bool, str) - (disponibilità, messaggio)
    """
    try:
        cap1 = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(1)

        if not cap1.isOpened() and not cap2.isOpened():
            return False, "Nessuna telecamera trovata"
        elif not cap1.isOpened():
            return False, "Telecamera sinistra non trovata"
        elif not cap2.isOpened():
            return False, "Telecamera destra non trovata"

        cap1.release()
        cap2.release()
        return True, "Telecamere OK"
    except Exception as e:
        return False, f"Errore nell'accesso alle telecamere: {str(e)}"

def check_model(model_path, device):
    """
    Verifica l'esistenza e la validità del modello.
    Returns: (bool, str) - (disponibilità, messaggio)
    """
    if not os.path.exists(model_path):
        return False, f"Modello non trovato in: {model_path}"

    try:
        # Prova a caricare il modello
        torch.load(model_path, map_location=device)
        return True, "Modello OK"
    except Exception as e:
        return False, f"Errore nel caricamento del modello: {str(e)}"
