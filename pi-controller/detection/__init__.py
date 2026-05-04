"""
K-Patrol Detection Package
==========================
Edge-AI safety pipeline for the K-Patrol patrol robot.

Pipeline (one process, three threads):

    [Pi Camera] --capture--> Queue --inference--> on_event() --> AlertBridge
                                                                     |
                                                                     +--> SQLite (durable)
                                                                     +--> MQTT (best-effort)
                                                                              |
                                                                              v
                                                              NavController.on_alert()
                                                              (fire -> EMERGENCY,
                                                               person -> 5s pause)

Components
----------
    AnomalyDetector     YOLOv8n (person, COCO class 0) + HSV fire segmentation.
                        Runs N-of-M temporal smoothing to suppress single-frame
                        flickers, emits DetectionEvent via on_event callback.

    AlertBridge        Glues the detector to MQTT. Persists every event to a
                        local SQLite write-ahead log first, then publishes to
                        kpatrol/{serial}/alert (QoS=1). A background drainer
                        retries unsynced rows whenever the broker comes back.
                        Survives cold-start broker outages and mid-publish
                        socket drops without losing events.

    AlertStore         SQLite write-ahead log (alerts.db). Append-only,
                        synced=0/1 column tracks broker delivery.

    TemporalSmoother   N-of-M consensus filter (default 3-of-5 frames). One
                        instance per kind ("person", "fire") inside the
                        detector.

Quick Start
-----------
    from detection import AnomalyDetector, DetectionConfig
    from detection.alert_bridge import AlertBridge, load_mqtt_env

    bridge = AlertBridge(load_mqtt_env(), db_path="alerts.db")
    bridge.connect()
    bridge.start_drainer()

    detector = AnomalyDetector(DetectionConfig(), on_event=bridge.publish)
    detector.start(blocking=True)
"""

from .anomaly_detector import AnomalyDetector, DetectionConfig, DetectionEvent

__all__ = ["AnomalyDetector", "DetectionConfig", "DetectionEvent"]
