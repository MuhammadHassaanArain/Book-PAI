import requests
import os
import xml.etree.ElementTree as ET
import trafilatura
import time
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
from dotenv import load_dotenv

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_KEY = os.getenv("QDRANT_KEY")
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))

SITEMAP_URL = "https://paihr.vercel.app/sitemap.xml"
COLLECTION_NAME = "book_content"

EMBED_MODEL = "embed-english-v3.0"
MAX_CHUNK_CHARS = 1200
MAX_PAGE_CHARS = 200_000   
EMBED_SLEEP = 1.6          

SKIP_PATTERNS = [
    "/docs/",
    "/docs/module-2/",
    "/docs/module-3/",
    "/docs/module-4/",
]

qdrant = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_KEY
)

# ------------------------
# Sitemap
# ------------------------
def get_all_urls(sitemap_url):
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    urls = []
    for child in root:
        loc = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc is not None:
            urls.append(loc.text.strip())

    return urls

# ------------------------
# Text extraction
# ------------------------
def extract_text_from_url(url):
    html = requests.get(url, timeout=15).text
    text = trafilatura.extract(html)

    if not text:
        print("[WARNING] No text:", url)
        return None

    text = text.strip()

    if len(text) < 500:
        print("[SKIPPED] Too small:", url)
        return None

    if len(text) > MAX_PAGE_CHARS:
        print("[SKIPPED] Page too large:", url, len(text))
        return None   # ✅ FIXED (previous bug)

    return text

# ------------------------
# Chunking
# ------------------------
def chunk_text(text, max_chars=MAX_CHUNK_CHARS):
    start = 0
    while start < len(text):
        end = min(start + max_chars, len(text))
        split_pos = text.rfind(". ", start, end)
        if split_pos == -1 or split_pos <= start:
            split_pos = end

        chunk = text[start:split_pos].strip()
        if chunk:
            yield chunk

        start = split_pos

# ------------------------
# Safe embedding (retry + sleep)
# ------------------------
def embed(text, retries=3):
    for attempt in range(retries):
        try:
            response = cohere_client.embed(
                model=EMBED_MODEL,
                input_type="search_query",
                texts=[text],
            )
            time.sleep(EMBED_SLEEP)   # ✅ RATE LIMIT
            return response.embeddings[0]

        except cohere.errors.TooManyRequestsError:
            wait = 5 * (attempt + 1)
            print(f"[RATE LIMIT] Sleeping {wait}s...")
            time.sleep(wait)

    print("[FAILED] Embed skipped after retries")
    return None

# ------------------------
# Qdrant
# ------------------------
def create_collection():
    if qdrant.collection_exists(COLLECTION_NAME):
        qdrant.delete_collection(COLLECTION_NAME)

    qdrant.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1024,
            distance=Distance.COSINE
        )
    )

def save_chunk_to_qdrant(chunk, chunk_id, url):
    vector = embed(chunk)
    if vector is None:
        return

    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "url": url,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )

# ------------------------
# Main ingest
# ------------------------
def ingest_book():
    urls = get_all_urls(SITEMAP_URL)
    create_collection()

    global_id = 1

    for url in urls:
        if any(url.rstrip("/").endswith(p.rstrip("/")) for p in SKIP_PATTERNS):
            print("[SKIPPED] Index:", url)
            continue

        print("\nProcessing:", url)

        try:
            text = extract_text_from_url(url)
        except Exception as e:
            print("[ERROR]", url, e)
            continue

        if not text:
            continue

        for chunk in chunk_text(text):
            save_chunk_to_qdrant(chunk, global_id, url)
            print(f"Saved chunk {global_id}")
            global_id += 1

if __name__ == "__main__":
    ingest_book()














# import requests
# import os
# import xml.etree.ElementTree as ET
# import trafilatura
# from qdrant_client import QdrantClient
# from qdrant_client.models import VectorParams, Distance, PointStruct
# import cohere
# from dotenv import load_dotenv
# load_dotenv()
# QDRANT_URL = os.getenv("QDRANT_URL")
# QDRANT_KEY = os.getenv("QDRANT_KEY")
# cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))


# SITEMAP_URL = "https://paihr.vercel.app/sitemap.xml"
# COLLECTION_NAME = "book_content"

# EMBED_MODEL = "embed-english-v3.0"
# MAX_CHUNK_CHARS = 1200
# MAX_PAGE_CHARS = 200_000

# SKIP_PATTERNS = [
#     "/docs/",
#     "/docs/module-2/",
#     "/docs/module-3/",
#     "/docs/module-4/",
# ]
# qdrant = QdrantClient(
#     url=QDRANT_URL,
#     api_key=QDRANT_KEY 
# )

# def get_all_urls(sitemap_url):
#     xml = requests.get(sitemap_url).text
#     root = ET.fromstring(xml)

#     urls = []
#     for child in root:
#         loc = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
#         if loc is not None:
#             urls.append(loc.text.strip())

#     print("\nFOUND URLS:")
#     for u in urls:
#         print(" -", u)

#     return urls

# def extract_text_from_url(url):
#     html = requests.get(url).text
#     text = trafilatura.extract(html)

#     if not text:
#         print("[WARNING] No text extracted from:", url)
#         return None
    
#     text = text.strip()
#     if len(text) < 500:
#         print("[SKIPPED] Too little content:", url)
#         return None
    
#     if len(text) > 300_000:
#         print("[SKIPPED] page too large:", url, len(text))
    
#     return text

# def chunk_text(text, max_chars=MAX_CHUNK_CHARS):
#     start = 0
#     text_len = len(text)

#     while start < text_len:
#         end = min(start + max_chars, text_len)
#         split_pos = text.rfind(". ", start, end)

#         if split_pos == -1 or split_pos <= start:
#             split_pos = end

#         chunk = text[start:split_pos].strip()
#         if chunk:
#             yield chunk
#         start = split_pos

# def embed(text):
#     response = cohere_client.embed(
#         model=EMBED_MODEL,
#         input_type="search_query",  
#         texts=[text],
#     )
#     return response.embeddings[0]  

# def create_collection():
#     if qdrant.collection_exists(COLLECTION_NAME):
#         qdrant.delete_collection(COLLECTION_NAME)

#     qdrant.create_collection(
#         collection_name=COLLECTION_NAME,
#         vectors_config=VectorParams(
#             size=1024,
#             distance=Distance.COSINE
#         )
#     )


# def save_chunk_to_qdrant(chunk, chunk_id, url):
#     vector = embed(chunk)

#     qdrant.upsert(
#         collection_name=COLLECTION_NAME,
#         points=[
#             PointStruct(
#                 id=chunk_id,
#                 vector=vector,
#                 payload={
#                     "url": url,
#                     "text": chunk,
#                     "chunk_id": chunk_id
#                 }
#             )
#         ]
#     )

# def ingest_book():
#     urls = get_all_urls(SITEMAP_URL)
#     create_collection()
#     global_id = 1
#     for url in urls:
#         # 🚫 Skip placeholder or invalid URLs
#         if "example.com" in url:
#             print("[SKIPPED] Placeholder URL:", url)
#             continue
#         if any(url.rstrip("/").endswith(p.rstrip("/")) for p in SKIP_PATTERNS):
#             print("[SKIPPED] Index page:", url)
#             continue

#         print("\nProcessing:", url)

#         try:
#             text = extract_text_from_url(url)
#         except Exception as e:
#             print("[ERROR] Failed to fetch:", url, e)
#             continue

#         if not text:
#             continue

#         for chunk in chunk_text(text):
#             save_chunk_to_qdrant(chunk, global_id, url)
#             print(f"Saved chunk {global_id}")
#             global_id += 1
        
# if __name__ == "__main__":
#     ingest_book()