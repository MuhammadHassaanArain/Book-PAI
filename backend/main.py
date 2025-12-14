import requests
import os
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
from dotenv import load_dotenv
load_dotenv()
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_KEY = os.getenv("QDRANT_KEY")


SITEMAP_URL = "https://paihr.vercel.app/sitemap.xml"
COLLECTION_NAME = "book_content"

cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
EMBED_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud
qdrant = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_KEY 
)


def get_all_urls(sitemap_url):
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print("\nFOUND URLS:")
    for u in urls:
        print(" -", u)

    return urls

def extract_text_from_url(url):
    html = requests.get(url).text
    text = trafilatura.extract(html)

    if not text:
        print("[WARNING] No text extracted from:", url)

    return text

def chunk_text(text, max_chars=1200):
    chunks = []
    while len(text) > max_chars:
        split_pos = text[:max_chars].rfind(". ")
        if split_pos == -1:
            split_pos = max_chars
        chunks.append(text[:split_pos])
        text = text[split_pos:]
    chunks.append(text)
    return chunks

def embed(text):
    response = cohere_client.embed(
        model=EMBED_MODEL,
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding

# def create_collection():
#     print("\nCreating Qdrant collection...")
#     qdrant.recreate_collection(
#         collection_name=COLLECTION_NAME,
#         vectors_config=VectorParams(
#         size=1024,        # Cohere embed-english-v3.0 dimension
#         distance=Distance.COSINE
#         )
#     )
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


# def ingest_book():
#     urls = get_all_urls(SITEMAP_URL)
#     create_collection()
#     global_id = 1
#     for url in urls:
#         print("\nProcessing:", url)
#         text = extract_text_from_url(url)
#         if not text:
#             continue
#         chunks = chunk_text(text)
#         for ch in chunks:
#             save_chunk_to_qdrant(ch, global_id, url)
#             print(f"Saved chunk {global_id}")
#             global_id += 1
#     print("\n✔️ Ingestion completed!")
#     print("Total chunks stored:", global_id - 1)

def ingest_book():
    urls = get_all_urls(SITEMAP_URL)
    create_collection()
    global_id = 1
    for url in urls:
        # 🚫 Skip placeholder or invalid URLs
        if "example.com" in url:
            print("[SKIPPED] Placeholder URL:", url)
            continue

        print("\nProcessing:", url)

        try:
            text = extract_text_from_url(url)
        except Exception as e:
            print("[ERROR] Failed to fetch:", url, e)
            continue

        if not text:
            continue

        chunks = chunk_text(text)

        for ch in chunks:
            save_chunk_to_qdrant(ch, global_id, url)
            print(f"Saved chunk {global_id}")
            global_id += 1

if __name__ == "__main__":
    ingest_book()