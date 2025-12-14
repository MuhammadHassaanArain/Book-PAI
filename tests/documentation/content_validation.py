#!/usr/bin/env python3
"""
Documentation Content Validation Script for Digital Twin System

This module provides validation tests for documentation content in the Digital Twin system.
It includes tests for content quality, APA citations, and technical accuracy.
"""

import os
import re
import unittest
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import yaml
import json
from dataclasses import dataclass


@dataclass
class ValidationResult:
    """
    Data class to hold validation results
    """
    is_valid: bool
    errors: List[str]
    warnings: List[str]
    info: List[str]


class DocumentationValidator:
    """
    Documentation validation class for testing content quality and consistency
    """

    def __init__(self, base_path: str = "docs/module-2"):
        """
        Initialize the DocumentationValidator

        Args:
            base_path (str): Base path for documentation files
        """
        self.base_path = Path(base_path)
        self.apa_pattern = re.compile(
            r'([A-Z][a-z]+,\s+[A-Z]\.\s*(?:\d{4})|"[^"]+"\s*\([^)]+\))'
        )
        self.code_block_pattern = re.compile(r'```.*?```', re.DOTALL)
        self.header_pattern = re.compile(r'^#+\s+(.+)', re.MULTILINE)
        self.image_pattern = re.compile(r'!\[([^\]]*)\]\(([^)]+)\)')
        self.link_pattern = re.compile(r'\[([^\]]+)\]\(([^)]+)\)')

    def validate_all_content(self) -> ValidationResult:
        """
        Validate all documentation content in the module

        Returns:
            ValidationResult: Complete validation results
        """
        errors = []
        warnings = []
        info = []

        # Get all markdown files
        md_files = list(self.base_path.rglob("*.md"))

        if not md_files:
            errors.append(f"No markdown files found in {self.base_path}")
            return ValidationResult(False, errors, warnings, info)

        info.append(f"Found {len(md_files)} markdown files to validate")

        # Validate each file
        for md_file in md_files:
            file_errors, file_warnings = self.validate_single_file(md_file)
            errors.extend(file_errors)
            warnings.extend(file_warnings)

        # Validate overall structure
        structure_errors = self.validate_directory_structure()
        errors.extend(structure_errors)

        # Validate citations
        citation_errors = self.validate_citations()
        errors.extend(citation_errors)

        is_valid = len(errors) == 0
        return ValidationResult(is_valid, errors, warnings, info)

    def validate_single_file(self, file_path: Path) -> Tuple[List[str], List[str]]:
        """
        Validate a single markdown file

        Args:
            file_path (Path): Path to the markdown file

        Returns:
            Tuple[List[str], List[str]]: Errors and warnings for the file
        """
        errors = []
        warnings = []

        try:
            content = file_path.read_text(encoding='utf-8')
        except Exception as e:
            errors.append(f"Could not read file {file_path}: {e}")
            return errors, warnings

        # Check file size (should not be empty)
        if len(content.strip()) == 0:
            errors.append(f"File is empty: {file_path}")
            return errors, warnings

        # Check for proper headers
        headers = self.header_pattern.findall(content)
        if not headers:
            warnings.append(f"No headers found in: {file_path}")
        else:
            # Check if first header is appropriate
            first_header = headers[0].strip()
            if len(first_header) < 3:
                warnings.append(f"First header seems too short in {file_path}: {first_header}")

        # Check for APA citations
        has_citations = self.check_apa_citations(content)
        if not has_citations:
            warnings.append(f"No APA citations found in: {file_path}")

        # Check for images
        images = self.image_pattern.findall(content)
        if images:
            for alt_text, img_path in images:
                # Check if image exists (if it's a relative path)
                if img_path.startswith('/assets/images/') or img_path.startswith('../assets/images/'):
                    expected_path = Path(img_path.replace('../', ''))
                    if not expected_path.exists():
                        errors.append(f"Image not found: {img_path} in {file_path}")

        # Check for links
        links = self.link_pattern.findall(content)
        for link_text, link_url in links:
            if link_url.startswith('http') and not link_url.startswith('http'):
                warnings.append(f"Potentially malformed link in {file_path}: {link_url}")

        # Check for code blocks
        code_blocks = self.code_block_pattern.findall(content)
        if code_blocks:
            for block in code_blocks:
                # Check if code block has language specified
                if not block.startswith('```python') and not block.startswith('```bash') and \
                   not block.startswith('```yaml') and not block.startswith('```text'):
                    warnings.append(f"Code block without language specification in {file_path}")

        # Check word count (should be substantial for educational content)
        words = len(content.split())
        if words < 200:
            warnings.append(f"File seems too short ({words} words): {file_path}")

        return errors, warnings

    def check_apa_citations(self, content: str) -> bool:
        """
        Check if content contains APA-style citations

        Args:
            content (str): Content to check

        Returns:
            bool: True if APA citations are found, False otherwise
        """
        # Look for common APA citation patterns
        patterns = [
            r'\([A-Z][a-z]+,\s*\d{4}\)',  # (Author, year)
            r'\([A-Z][a-z]+,\s*\d{4};[^\)]+\)',  # (Author, year; Author, year)
            r'[A-Z][a-z]+\s*\(\d{4}\)',  # Author (year)
            r'[A-Z][a-z]+\s*et\s*al\.,\s*\d{4}'  # Author et al., year
        ]

        for pattern in patterns:
            if re.search(pattern, content):
                return True

        return False

    def validate_directory_structure(self) -> List[str]:
        """
        Validate the expected directory structure

        Returns:
            List[str]: Errors found in directory structure
        """
        errors = []

        expected_dirs = [
            self.base_path / "chapter-1-gazebo-physics",
            self.base_path / "chapter-2-world-building",
            self.base_path / "chapter-3-sensor-simulation",
            self.base_path / "chapter-4-unity-visualization",
            self.base_path / "labs",
            self.base_path / "mini-projects",
            self.base_path / "references",
            self.base_path / "tutorials"
        ]

        for expected_dir in expected_dirs:
            if not expected_dir.exists():
                errors.append(f"Missing expected directory: {expected_dir}")

        # Check for expected files in each chapter
        chapter_dirs = [d for d in expected_dirs if "chapter" in str(d)]
        for chapter_dir in chapter_dirs:
            if chapter_dir.exists():
                md_files = list(chapter_dir.glob("*.md"))
                if len(md_files) == 0:
                    errors.append(f"Chapter directory {chapter_dir} is empty")

        return errors

    def validate_citations(self) -> List[str]:
        """
        Validate that citations follow APA 7th edition format

        Returns:
            List[str]: Citation-related errors
        """
        errors = []

        # Look for bibliography or reference files
        ref_files = list(self.base_path.glob("references/*.md")) + \
                   list(self.base_path.glob("*.md"))

        citation_found = False
        for ref_file in ref_files:
            try:
                content = ref_file.read_text(encoding='utf-8')
                if self.check_apa_citations(content):
                    citation_found = True
                    break
            except Exception:
                continue

        if not citation_found:
            errors.append("No APA citations found in documentation")

        return errors

    def validate_content_accuracy(self, expected_topics: List[str] = None) -> ValidationResult:
        """
        Validate that content covers expected topics

        Args:
            expected_topics (List[str]): List of topics that should be covered

        Returns:
            ValidationResult: Validation results for content accuracy
        """
        if expected_topics is None:
            expected_topics = [
                "Gazebo", "Unity", "ROS 2", "physics", "simulation",
                "sensor", "LiDAR", "IMU", "digital twin", "humanoid"
            ]

        errors = []
        warnings = []
        info = []

        # Search for expected topics in all markdown files
        md_files = list(self.base_path.rglob("*.md"))
        found_topics = []

        for md_file in md_files:
            try:
                content = md_file.read_text(encoding='utf-8').lower()
                for topic in expected_topics:
                    if topic.lower() in content:
                        if topic not in found_topics:
                            found_topics.append(topic)
            except Exception as e:
                errors.append(f"Could not read {md_file}: {e}")

        missing_topics = [topic for topic in expected_topics if topic not in found_topics]
        if missing_topics:
            warnings.append(f"Missing topics in documentation: {missing_topics}")
        else:
            info.append(f"All expected topics found: {found_topics}")

        is_valid = len(missing_topics) <= len(expected_topics) * 0.2  # Allow up to 20% missing
        return ValidationResult(is_valid, errors, warnings, info)

    def validate_word_count(self, min_words: int = 7000, max_words: int = 9000) -> ValidationResult:
        """
        Validate that total word count is within expected range

        Args:
            min_words (int): Minimum expected word count
            max_words (int): Maximum expected word count

        Returns:
            ValidationResult: Validation results for word count
        """
        errors = []
        warnings = []
        info = []

        total_words = 0
        md_files = list(self.base_path.rglob("*.md"))

        for md_file in md_files:
            try:
                content = md_file.read_text(encoding='utf-8')
                # Remove code blocks and headers for word count
                clean_content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
                clean_content = re.sub(r'^#+.*$', '', clean_content, flags=re.MULTILINE)
                words = len(clean_content.split())
                total_words += words
            except Exception as e:
                errors.append(f"Could not count words in {md_file}: {e}")

        info.append(f"Total word count: {total_words}")

        if total_words < min_words:
            errors.append(f"Total word count ({total_words}) is below minimum ({min_words})")
        elif total_words > max_words:
            errors.append(f"Total word count ({total_words}) exceeds maximum ({max_words})")

        is_valid = min_words <= total_words <= max_words
        return ValidationResult(is_valid, errors, warnings, info)


def main():
    """
    Main function to run documentation validation tests
    """
    print("Running Documentation Validation for Digital Twin Module...")

    validator = DocumentationValidator()

    # Run comprehensive validation
    all_results = validator.validate_all_content()

    # Run specific validations
    accuracy_results = validator.validate_content_accuracy()
    word_count_results = validator.validate_word_count()

    # Print results
    print("\n=== Documentation Validation Results ===")
    print(f"All Content Validation: {'PASS' if all_results.is_valid else 'FAIL'}")
    print(f"Accuracy Validation: {'PASS' if accuracy_results.is_valid else 'FAIL'}")
    print(f"Word Count Validation: {'PASS' if word_count_results.is_valid else 'FAIL'}")

    if all_results.errors:
        print(f"\nErrors found: {len(all_results.errors)}")
        for error in all_results.errors:
            print(f"  - {error}")

    if all_results.warnings:
        print(f"\nWarnings: {len(all_results.warnings)}")
        for warning in all_results.warnings:
            print(f"  - {warning}")

    if accuracy_results.errors:
        print(f"\nAccuracy Errors: {len(accuracy_results.errors)}")
        for error in accuracy_results.errors:
            print(f"  - {error}")

    if word_count_results.errors:
        print(f"\nWord Count Issues: {len(word_count_results.errors)}")
        for error in word_count_results.errors:
            print(f"  - {error}")

    overall_success = all_results.is_valid and accuracy_results.is_valid and word_count_results.is_valid
    print(f"\nOverall: {'PASS' if overall_success else 'FAIL'}")

    return overall_success


class TestDocumentationValidation(unittest.TestCase):
    """
    Unit tests for documentation validation functions
    """

    def setUp(self):
        """
        Set up the test case
        """
        self.validator = DocumentationValidator()

    def test_validate_directory_structure(self):
        """
        Test directory structure validation
        """
        errors = self.validator.validate_directory_structure()
        # This test checks that the method runs without error
        self.assertIsInstance(errors, list)

    def test_check_apa_citations(self):
        """
        Test APA citation detection
        """
        content_with_citation = "This is based on research (Smith, 2023)."
        has_citation = self.validator.check_apa_citations(content_with_citation)
        self.assertTrue(has_citation)

        content_without_citation = "This has no citations."
        has_no_citation = self.validator.check_apa_citations(content_without_citation)
        self.assertFalse(has_no_citation)

    def test_validate_content_accuracy(self):
        """
        Test content accuracy validation
        """
        result = self.validator.validate_content_accuracy(["Gazebo", "simulation"])
        self.assertIsInstance(result, ValidationResult)


if __name__ == '__main__':
    # Run the main validation
    success = main()

    # If running as part of a test suite, also run unit tests
    if success:
        unittest.main(argv=[''], exit=False, verbosity=2)