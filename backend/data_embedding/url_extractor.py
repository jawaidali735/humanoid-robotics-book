
import requests
import xml.etree.ElementTree as ET
from typing import List
import logging
import os
import sys

# Add the parent directory to the path to import from config
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.settings import SITEMAP_URL

logger = logging.getLogger(__name__)

def get_all_urls(sitemap_url: str = None) -> List[str]:
    """
    Extract all URLs from a sitemap XML.

    Args:
        sitemap_url (str): URL of the sitemap to process. If None, uses default from config.

    Returns:
        List[str]: List of URLs extracted from the sitemap
    """
    if sitemap_url is None:
        sitemap_url = SITEMAP_URL

    logger.info(f"Fetching sitemap from {sitemap_url}")
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    logger.info(f"Found {len(urls)} URLs in sitemap")
    for u in urls:
        logger.info(f" - {u}")

    return urls


def extract_text_from_url(url: str) -> str:
    """
    Extract text content from a given URL.

    Args:
        url (str): URL to extract text from

    Returns:
        str: Extracted text content
    """
    import requests
    from bs4 import BeautifulSoup

    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }
        response = requests.get(url, headers=headers, timeout=30)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        text = soup.get_text()

        # Clean up the text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        logger.info(f"Successfully extracted {len(text)} characters from {url}")
        return text

    except Exception as e:
        logger.error(f"Error extracting text from {url}: {str(e)}")
        return ""