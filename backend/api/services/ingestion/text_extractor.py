"""HTML text extraction from Docusaurus pages."""

import re
from html.parser import HTMLParser
from ...utils import get_logger

logger = get_logger(__name__)


class DocusaurusTextExtractor(HTMLParser):
    """Extract clean text from Docusaurus HTML pages."""

    # Tags to completely ignore (including their content)
    IGNORED_TAGS: set[str] = {
        "script",
        "style",
        "nav",
        "header",
        "footer",
        "aside",
        "noscript",
        "svg",
        "button",
        "form",
        "input",
        "select",
        "textarea",
    }

    # Void tags that don't have a closing tag
    VOID_TAGS: set[str] = {
        "area", "base", "br", "col", "embed", "hr", "img", "input",
        "link", "meta", "param", "source", "track", "wbr"
    }

    # Class patterns that indicate navigation/non-content areas
    IGNORED_CLASS_PATTERNS: list[str] = [
        "theme-layout-navbar",
        "theme-doc-sidebar",
        "footer",
        "breadcrumb",
        "pagination",
        "table-of-contents",
        "edit-this-page",
        "theme-toggle",
    ]

    def __init__(self) -> None:
        """Initialize HTML parser."""
        super().__init__()
        self.text_parts: list[str] = []
        # Stack to track ignore status. None means not ignored.
        # String means ignored due to that reason.
        self.ignore_stack: list[str | None] = [None]
        self.current_tag: str = ""

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        """Handle opening tags."""
        self.current_tag = tag.lower()
        
        # Don't track void tags in stack as they don't have end tags
        if self.current_tag in self.VOID_TAGS:
            return

        parent_reason = self.ignore_stack[-1] if self.ignore_stack else None
        
        # If parent is ignored, this tag is automatically ignored (inherit reason)
        if parent_reason:
            self.ignore_stack.append(parent_reason)
            return

        # Check if we should start ignoring this tag
        ignore_reason = None
        
        # Check tag name
        if self.current_tag in self.IGNORED_TAGS:
            ignore_reason = f"tag:{tag}"
        else:
            # Check class attributes for navigation patterns
            attrs_dict = dict(attrs)
            class_attr = attrs_dict.get("class", "") or ""
            id_attr = attrs_dict.get("id", "") or ""

            for pattern in self.IGNORED_CLASS_PATTERNS:
                if pattern in class_attr.lower() or pattern in id_attr.lower():
                    ignore_reason = f"class_pattern:{pattern} in '{class_attr}'"
                    break
        
        self.ignore_stack.append(ignore_reason)

    def handle_endtag(self, tag: str) -> None:
        """Handle closing tags."""
        if tag.lower() in self.VOID_TAGS:
            return
            
        if len(self.ignore_stack) > 1:  # Keep the root
            self.ignore_stack.pop()

    def handle_data(self, data: str) -> None:
        """Extract text data."""
        # If current level is ignored, skip data
        if self.ignore_stack and self.ignore_stack[-1]:
            return

        cleaned = data.strip()
        if cleaned:
            # Add paragraph breaks after certain block elements
            if self.current_tag in ("p", "div", "h1", "h2", "h3", "h4", "h5", "h6", "li"):
                self.text_parts.append(cleaned + "\n")
            else:
                self.text_parts.append(cleaned + " ")

    def get_text(self) -> str:
        """Get extracted and cleaned text."""
        raw_text = "".join(self.text_parts)

        # Normalize whitespace
        text = re.sub(r"\s+", " ", raw_text)

        # Preserve paragraph structure
        text = re.sub(r"\n\s*\n", "\n\n", text)

        # Remove common Docusaurus artifacts
        artifacts = [
            "Skip to main content",
            "Edit this page",
            "Last updated",
            "Previous",
            "Next",
            "On this page",
            "Copyright",
        ]
        for artifact in artifacts:
            text = text.replace(artifact, "")

        return text.strip()


def extract_text_from_html(html_content: str) -> str:
    """Extract clean text from HTML.

    Args:
        html_content: Raw HTML from a Docusaurus page

    Returns:
        Cleaned text suitable for embedding
    """
    parser = DocusaurusTextExtractor()
    try:
        parser.feed(html_content)
        return parser.get_text()
    except Exception as e:
        logger.warning(f"HTML parsing error: {e}")
        # Fallback: basic regex-based extraction
        text = re.sub(r"<[^>]+>", " ", html_content)
        text = re.sub(r"\s+", " ", text)
        return text.strip()
