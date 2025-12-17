"""
Metrics tracking for RAG Chatbot performance.

Tracks latency, accuracy, and other key metrics for queries and operations.
"""

import time
import json
from pathlib import Path
from typing import Optional, Dict, Any
from datetime import datetime
from dataclasses import dataclass, asdict
import threading


@dataclass
class QueryMetrics:
    """Metrics for a single query execution."""

    query_id: str
    book_id: str
    query_text: str
    retrieval_latency_ms: float
    generation_latency_ms: float
    total_latency_ms: float
    confidence: float
    chunks_retrieved: int
    chunks_reranked: int
    citations_count: int
    response_length: int
    timestamp: str


@dataclass
class AccuracyMetrics:
    """Metrics for accuracy evaluation."""

    query_id: str
    expected_answer: str
    generated_answer: str
    accuracy_score: float
    retrieval_precision: float
    citation_accuracy: float
    timestamp: str


class LatencyTracker:
    """Context manager for measuring operation latency."""

    def __init__(self):
        """Initialize latency tracker."""
        self.start_time = None
        self.end_time = None

    def __enter__(self):
        """Start timer."""
        self.start_time = time.time()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Stop timer."""
        self.end_time = time.time()

    @property
    def elapsed_ms(self) -> float:
        """Get elapsed time in milliseconds."""
        if self.start_time is None:
            return 0.0
        end = self.end_time or time.time()
        return (end - self.start_time) * 1000


class MetricsRecorder:
    """Records and persists query and accuracy metrics."""

    def __init__(self, metrics_file: str = "./metrics/accuracy.json"):
        """
        Initialize metrics recorder.

        Args:
            metrics_file: Path to store metrics JSON
        """
        self.metrics_file = Path(metrics_file)
        self.metrics_file.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def record_query_metrics(self, metrics: QueryMetrics) -> None:
        """
        Record query execution metrics.

        Args:
            metrics: QueryMetrics dataclass with execution data
        """
        with self._lock:
            metrics_list = self._load_metrics()
            metrics_list.append(asdict(metrics))
            self._save_metrics(metrics_list)

    def record_accuracy_metrics(self, metrics: AccuracyMetrics) -> None:
        """
        Record accuracy evaluation metrics.

        Args:
            metrics: AccuracyMetrics dataclass with evaluation data
        """
        with self._lock:
            metrics_list = self._load_metrics()
            metrics_list.append(asdict(metrics))
            self._save_metrics(metrics_list)

    def get_query_metrics(self, query_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve recorded metrics for a query.

        Args:
            query_id: Query identifier

        Returns:
            Metrics dict or None if not found
        """
        metrics_list = self._load_metrics()
        for metric in metrics_list:
            if metric.get("query_id") == query_id:
                return metric
        return None

    def get_average_latency(self, limit: int = 100) -> Dict[str, float]:
        """
        Calculate average latency metrics across recent queries.

        Args:
            limit: Number of recent metrics to consider

        Returns:
            Dict with average latencies
        """
        metrics_list = self._load_metrics()
        query_metrics = [
            m for m in metrics_list[-limit:] if "total_latency_ms" in m
        ]

        if not query_metrics:
            return {
                "avg_total_latency_ms": 0.0,
                "avg_retrieval_latency_ms": 0.0,
                "avg_generation_latency_ms": 0.0,
            }

        avg_total = sum(m.get("total_latency_ms", 0) for m in query_metrics) / len(
            query_metrics
        )
        avg_retrieval = sum(m.get("retrieval_latency_ms", 0) for m in query_metrics) / len(
            query_metrics
        )
        avg_generation = sum(m.get("generation_latency_ms", 0) for m in query_metrics) / len(
            query_metrics
        )

        return {
            "avg_total_latency_ms": round(avg_total, 2),
            "avg_retrieval_latency_ms": round(avg_retrieval, 2),
            "avg_generation_latency_ms": round(avg_generation, 2),
        }

    def get_average_accuracy(self, limit: int = 100) -> Dict[str, float]:
        """
        Calculate average accuracy across evaluated queries.

        Args:
            limit: Number of recent evaluations to consider

        Returns:
            Dict with average accuracy scores
        """
        metrics_list = self._load_metrics()
        accuracy_metrics = [m for m in metrics_list[-limit:] if "accuracy_score" in m]

        if not accuracy_metrics:
            return {
                "avg_accuracy": 0.0,
                "avg_retrieval_precision": 0.0,
                "avg_citation_accuracy": 0.0,
            }

        avg_accuracy = sum(m.get("accuracy_score", 0) for m in accuracy_metrics) / len(
            accuracy_metrics
        )
        avg_retrieval = sum(m.get("retrieval_precision", 0) for m in accuracy_metrics) / len(
            accuracy_metrics
        )
        avg_citation = sum(m.get("citation_accuracy", 0) for m in accuracy_metrics) / len(
            accuracy_metrics
        )

        return {
            "avg_accuracy": round(avg_accuracy, 3),
            "avg_retrieval_precision": round(avg_retrieval, 3),
            "avg_citation_accuracy": round(avg_citation, 3),
        }

    def _load_metrics(self) -> list:
        """Load metrics from file."""
        if not self.metrics_file.exists():
            return []
        try:
            with open(self.metrics_file, "r") as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
            return []

    def _save_metrics(self, metrics_list: list) -> None:
        """Save metrics to file."""
        with open(self.metrics_file, "w") as f:
            json.dump(metrics_list, f, indent=2)


# Global metrics recorder instance
_recorder_instance: Optional[MetricsRecorder] = None


def get_metrics_recorder(metrics_file: str = "./metrics/accuracy.json") -> MetricsRecorder:
    """
    Get or create global metrics recorder instance.

    Args:
        metrics_file: Path to metrics storage file

    Returns:
        MetricsRecorder instance
    """
    global _recorder_instance
    if _recorder_instance is None:
        _recorder_instance = MetricsRecorder(metrics_file)
    return _recorder_instance
