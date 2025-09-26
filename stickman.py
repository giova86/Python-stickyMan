# libraries
import cv2
import time
import mediapipe as mp
import numpy as np
import os
import argparse
import signal
import sys
from utils import mediapipe_detection, draw_landmarks, draw_landmarks_custom, draw_limit_rh, draw_limit_lh, check_detection

import pyvirtualcam
from pyvirtualcam import PixelFormat

import objc
from AVFoundation import AVCaptureDevice

# Variabili globali per la gestione della chiusura
cap = None
virtual_cam = None
running = True

def get_webcam_names():
    webcam_list = []
    devices = AVCaptureDevice.devicesWithMediaType_("vide")

    for device in devices:
        name = device.localizedName()
        webcam_list.append(name)

    return webcam_list

def print_banner():
    """Stampa il banner di avvio dell'applicazione"""
    print("\n" + "="*60)
    print("🤖 STICKY MAN - Motion Capture to Stick Figure")
    print("="*60)

def print_success(message):
    """Stampa un messaggio di successo"""
    print(f"✅ {message}")

def print_error(message):
    """Stampa un messaggio di errore"""
    print(f"❌ ERRORE: {message}")

def print_warning(message):
    """Stampa un messaggio di warning"""
    print(f"⚠️  WARNING: {message}")

def print_info(message):
    """Stampa un messaggio informativo"""
    print(f"ℹ️  {message}")

def print_separator():
    """Stampa una linea separatrice"""
    print("-" * 60)

def signal_handler(sig, frame):
    """Gestisce l'interruzione da Ctrl+C"""
    global running, cap, virtual_cam
    print_info("\nChiusura in corso...")
    running = False

    if cap:
        cap.release()
    if virtual_cam:
        virtual_cam.close()
    cv2.destroyAllWindows()

    print_success("Applicazione chiusa correttamente")
    sys.exit(0)
    webcam_list = []
    devices = AVCaptureDevice.devicesWithMediaType_("vide")

    for device in devices:
        name = device.localizedName()
        webcam_list.append(name)

    return webcam_list

def list_webcams():
    """Elenca tutte le webcam disponibili"""
    devices = get_webcam_names()
    print("Webcam disponibili:")
    for i, device in enumerate(devices):
        print(f"  {i}: {device}")
    return devices

def open_webcam(webcam_id, output_mode):
    """
    Apre la webcam e processa il video

    Args:
        webcam_id (int): ID della webcam da utilizzare
        output_mode (str): 'window', 'virtual', o 'both'
    """
    global cap, virtual_cam, running

    mp_holistic = mp.solutions.holistic
    mp_drawing = mp.solutions.drawing_utils

    # Registra il signal handler per Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    print_banner()

    width = 1920
    height = 1080
    cap = cv2.VideoCapture(webcam_id)

    if not cap.isOpened():
        print_error(f"Impossibile aprire la webcam con ID {webcam_id}")
        return

    print(f"\n - Webcam {webcam_id} aperta correttamente\n")

    # Inizializza la virtual camera solo se necessario
    virtual_cam = None
    if output_mode in ['virtual', 'both']:
        try:
            virtual_cam = pyvirtualcam.Camera(width, height, 30, fmt=PixelFormat.BGR)
            print_success(f"Virtual camera inizializzata: {virtual_cam.device}")
        except Exception as e:
            print_error(f"Inizializzazione virtual camera fallita: {e}")
            if output_mode == 'virtual':
                return
            print_warning("Fallback alla modalità finestra")
            output_mode = 'window'

    print_separator()
    print(f"\n - Modalità output: {output_mode.upper()}")
    print(" - Controlli:")
    print("   • Premi 'q' per uscire")
    print("   • Premi 'Ctrl+C' per uscire\n")
    print_separator()

    try:
        with mp_holistic.Holistic(min_detection_confidence=0.5,
                                  min_tracking_confidence=0.5) as holistic:

            frame_count = 0
            start_time = time.time()

            while cap.isOpened() and running:
                ret, frame = cap.read()
                if not ret:
                    print_error("Impossibile leggere il frame dalla webcam")
                    break

                # Make detection
                image, results = mediapipe_detection(frame, holistic)

                # Crea l'immagine stick figure
                img = np.zeros([frame.shape[0], frame.shape[1], 3], dtype=np.uint8)
                draw_landmarks_custom(img, results)

                # Output su finestra
                if output_mode in ['window', 'both']:
                    cv2.imshow('Sticky Man - Premi Q per uscire', img)

                # Output su virtual camera
                if output_mode in ['virtual', 'both'] and virtual_cam:
                    virtual_cam.send(img)
                    virtual_cam.sleep_until_next_frame()

                # # Calcolo FPS ogni 30 frames
                # frame_count += 1
                # if frame_count % 30 == 0:
                #     elapsed_time = time.time() - start_time
                #     fps = 30 / elapsed_time
                #     print(f"\n - 🎥 FPS: {fps:.1f} | Frame: {frame_count} | Premi 'q' o 'Ctrl+C' per uscire", end="", flush=True)
                #     start_time = time.time()

                # Controllo per uscire con 'q'
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\n - Uscita richiesta dall'utente (tasto 'q')")
                    break

    except KeyboardInterrupt:
        print("\n - Interruzione da Ctrl+C")
    except Exception as e:
        print_error(f"Errore durante l'esecuzione: {e}")
    finally:
        cleanup()

def cleanup():
    """Pulizia delle risorse"""
    global cap, virtual_cam

    print(" - Pulizia risorse in corso...")

    if cap:
        cap.release()
        print(" - ✅ Webcam rilasciata")

    if virtual_cam:
        virtual_cam.close()
        print(" - ✅ Virtual camera chiusa")

    cv2.destroyAllWindows()
    print(" - ✅ Finestre chiuse\n")
    print_separator()

def main():
    parser = argparse.ArgumentParser(
        description='🤖 Sticky Man - Motion capture to stick figure',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Esempi di utilizzo:
  python stickman.py -l                    # Elenca webcam disponibili
  python stickman.py                       # Usa webcam 0 con finestra
  python stickman.py -w 1 -o virtual      # Webcam 1 con virtual camera
  python stickman.py -w 0 -o both         # Webcam 0 con finestra + virtual

Controlli durante l'esecuzione:
  q        # Esci dall'applicazione
  Ctrl+C   # Interrompi l'applicazione
        """
    )
    parser.add_argument('-w', '--webcam', type=int, default=0,
                        help='ID della webcam da utilizzare (default: 0)')
    parser.add_argument('-o', '--output', choices=['window', 'virtual', 'both'],
                        default='window',
                        help='Modalità di output: window (finestra), virtual (virtual camera), both (entrambi)')
    parser.add_argument('-l', '--list', action='store_true',
                        help='Elenca tutte le webcam disponibili')

    args = parser.parse_args()

    if args.list:
        list_webcams()
        return

    # Verifica che la webcam esista
    devices = get_webcam_names()
    if not devices:
        print_error("Nessuna webcam trovata nel sistema")
        return

    if args.webcam >= len(devices):
        print_error(f"Webcam ID {args.webcam} non trovata")
        print_separator()
        print_info("Webcam disponibili:")
        for i, device in enumerate(devices):
            print(f"  📷 {i}: {device}")
        print_separator()
        return

    print_info(f"Webcam selezionata: {devices[args.webcam]} (ID: {args.webcam})")
    open_webcam(args.webcam, args.output)

if __name__ == "__main__":
    main()
