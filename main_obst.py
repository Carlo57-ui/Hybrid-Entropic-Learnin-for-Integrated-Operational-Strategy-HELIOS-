from obstacle_detector import ObstacleDetector
import time

def main():
    detector = ObstacleDetector(max_distance=2000)  # 2 m
    print("Iniciando detección de obstáculos... Presiona Ctrl+C para detener.")

    try:
        while True:
            obstacles = detector.get_obstacles()
            print(f"Se detectaron {len(obstacles)} obstáculos")

            for i, (X, Z) in enumerate(obstacles):
                print(f"  Obstáculo {i+1}: X={X:.2f} m, Z={Z:.2f} m")

            time.sleep(0.5)  # medio segundo entre lecturas

    except KeyboardInterrupt:
        print("Detenido por el usuario.")

if __name__ == "__main__":
    main()
