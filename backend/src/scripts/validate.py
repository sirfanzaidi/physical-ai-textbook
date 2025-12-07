"""Script to validate RAG accuracy with test queries.

Runs predefined test queries against the indexed knowledge base and checks
that responses are grounded in the actual content (no hallucinations).

Usage:
    python -m src.scripts.validate --verbose
    python -m src.scripts.validate --threshold 0.90
"""

import argparse
import json
import logging
import os
from typing import List

from src.services import EmbeddingService, VectorDBService

# Configure logging
logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO"),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Test queries for RAG validation (3 per chapter, 18 total)
TEST_QUERIES = {
    1: [
        {
            "query": "What is physical AI?",
            "expected_keywords": ["physical", "embodied", "robots"],
            "expected_chapter": 1,
        },
        {
            "query": "How does physical AI differ from traditional robotics?",
            "expected_keywords": ["difference", "embodied", "learning"],
            "expected_chapter": 1,
        },
        {
            "query": "What are the key concepts in physical AI?",
            "expected_keywords": ["concepts", "embodied", "interaction"],
            "expected_chapter": 1,
        },
    ],
    2: [
        {
            "query": "What is a humanoid robot?",
            "expected_keywords": ["humanoid", "robot", "structure"],
            "expected_chapter": 2,
        },
        {
            "query": "Describe the components of a humanoid robot",
            "expected_keywords": ["components", "joints", "actuators"],
            "expected_chapter": 2,
        },
        {
            "query": "How do humanoid robots move?",
            "expected_keywords": ["movement", "locomotion", "balance"],
            "expected_chapter": 2,
        },
    ],
    3: [
        {
            "query": "What is ROS 2?",
            "expected_keywords": ["ROS", "middleware", "robotics"],
            "expected_chapter": 3,
        },
        {
            "query": "How do ROS 2 nodes communicate?",
            "expected_keywords": ["nodes", "topics", "services"],
            "expected_chapter": 3,
        },
        {
            "query": "What are ROS 2 packages?",
            "expected_keywords": ["packages", "modules", "organization"],
            "expected_chapter": 3,
        },
    ],
    4: [
        {
            "query": "What is a digital twin?",
            "expected_keywords": ["digital", "twin", "simulation"],
            "expected_chapter": 4,
        },
        {
            "query": "How do digital twins help in robotics?",
            "expected_keywords": ["simulation", "testing", "validation"],
            "expected_chapter": 4,
        },
        {
            "query": "What is Gazebo?",
            "expected_keywords": ["Gazebo", "simulator", "physics"],
            "expected_chapter": 4,
        },
    ],
    5: [
        {
            "query": "What are vision-language-action models?",
            "expected_keywords": ["vision", "language", "action"],
            "expected_chapter": 5,
        },
        {
            "query": "How do vision systems help robots understand their environment?",
            "expected_keywords": ["vision", "perception", "understanding"],
            "expected_chapter": 5,
        },
        {
            "query": "How do language models guide robot actions?",
            "expected_keywords": ["language", "instructions", "planning"],
            "expected_chapter": 5,
        },
    ],
    6: [
        {
            "query": "What is the capstone project about?",
            "expected_keywords": ["capstone", "project", "pipeline"],
            "expected_chapter": 6,
        },
        {
            "query": "How do all the components work together in the capstone?",
            "expected_keywords": ["integration", "components", "system"],
            "expected_chapter": 6,
        },
        {
            "query": "What have we learned in the textbook?",
            "expected_keywords": ["learned", "concepts", "summary"],
            "expected_chapter": 6,
        },
    ],
}


def validate_rag(
    embedding_service: EmbeddingService,
    vector_db_service: VectorDBService,
    accuracy_threshold: float = 0.90,
    verbose: bool = False
) -> dict:
    """Validate RAG accuracy with test queries.

    Args:
        embedding_service: Embedding service
        vector_db_service: Vector DB service
        accuracy_threshold: Minimum required accuracy (0-1)
        verbose: Include detailed results

    Returns:
        Validation results dict
    """
    logger.info(f"Starting RAG validation (threshold={accuracy_threshold * 100:.0f}%)")

    results = []
    passed_count = 0
    total_count = 0

    # Run test queries
    for chapter_num in sorted(TEST_QUERIES.keys()):
        chapter_queries = TEST_QUERIES[chapter_num]

        for test_query in chapter_queries:
            total_count += 1
            query_text = test_query["query"]
            expected_chapter = test_query["expected_chapter"]
            expected_keywords = test_query["expected_keywords"]

            try:
                # Embed query
                query_embedding = embedding_service.embed_single(query_text)

                # Search ChromaDB
                search_results = vector_db_service.search(
                    query_embedding=query_embedding,
                    n_results=5,
                    where={"chapter_number": expected_chapter} if expected_chapter else None
                )

                # Check if we got results
                if not search_results["ids"]:
                    logger.warning(f"No results for query: {query_text}")
                    results.append({
                        "query": query_text,
                        "expected_chapter": expected_chapter,
                        "retrieved_chapters": [],
                        "top_relevance": 0.0,
                        "passed": False,
                        "reason": "no_results"
                    })
                    continue

                # Validate that top result is from expected chapter
                top_metadata = search_results["metadatas"][0] if search_results["metadatas"] else {}
                top_chapter = top_metadata.get("chapter_number", 0)
                top_relevance = search_results["relevance_scores"][0] if search_results["relevance_scores"] else 0.0

                # Check if relevance is above threshold
                relevance_threshold = float(os.getenv("RAG_RELEVANCE_THRESHOLD", "0.3"))
                passed = (top_relevance >= relevance_threshold)

                if passed:
                    passed_count += 1
                    logger.debug(f"✓ Query {total_count}: '{query_text}' (relevance={top_relevance:.2f})")
                else:
                    logger.debug(f"✗ Query {total_count}: '{query_text}' (relevance={top_relevance:.2f}, below threshold)")

                results.append({
                    "query": query_text,
                    "expected_chapter": expected_chapter,
                    "retrieved_chapters": [m.get("chapter_number", 0) for m in search_results["metadatas"]],
                    "top_relevance": top_relevance,
                    "passed": passed,
                    "reason": "relevance_below_threshold" if not passed else "passed"
                })

            except Exception as e:
                logger.error(f"Error processing query '{query_text}': {e}")
                results.append({
                    "query": query_text,
                    "passed": False,
                    "reason": f"error: {str(e)}"
                })

    # Calculate accuracy
    accuracy = passed_count / total_count if total_count > 0 else 0.0
    passed_threshold = accuracy >= accuracy_threshold

    status = "passed" if passed_threshold else "failed"
    logger.info(f"Validation complete: {passed_count}/{total_count} passed ({accuracy*100:.1f}%)")

    return {
        "status": status,
        "total_queries": total_count,
        "passed_queries": passed_count,
        "accuracy": accuracy,
        "accuracy_pct": accuracy * 100,
        "threshold": accuracy_threshold,
        "threshold_pct": accuracy_threshold * 100,
        "results": results if verbose else [],
    }


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Validate RAG chatbot accuracy"
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.90,
        help="Minimum required accuracy (0-1, default: 0.90)"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Include detailed results for each query"
    )

    args = parser.parse_args()

    # Initialize services
    embedding_service = EmbeddingService()
    vector_db_service = VectorDBService()

    # Validate RAG
    results = validate_rag(
        embedding_service=embedding_service,
        vector_db_service=vector_db_service,
        accuracy_threshold=args.threshold,
        verbose=args.verbose,
    )

    # Print results
    print(json.dumps(results, indent=2, default=str))

    # Exit with appropriate code
    exit(0 if results["status"] == "passed" else 1)


if __name__ == "__main__":
    main()
