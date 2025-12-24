import hashlib
import time
from typing import Optional, Dict, Any
from datetime import datetime, timedelta

class SimpleCache:
    """
    Simple in-memory cache for storing translation results
    """
    def __init__(self, default_ttl: int = 3600):  # 1 hour default TTL
        self._cache: Dict[str, Dict[str, Any]] = {}
        self.default_ttl = default_ttl

    def _generate_key(self, text: str, source_lang: str, target_lang: str) -> str:
        """
        Generate a unique key for the cache based on the translation request
        """
        key_string = f"{text}:{source_lang}:{target_lang}"
        return hashlib.md5(key_string.encode()).hexdigest()

    def get(self, text: str, source_lang: str, target_lang: str) -> Optional[Dict[str, Any]]:
        """
        Get cached translation result if it exists and hasn't expired
        """
        key = self._generate_key(text, source_lang, target_lang)

        if key in self._cache:
            cached_item = self._cache[key]
            if time.time() < cached_item['expires_at']:
                return cached_item['data']
            else:
                # Remove expired item
                del self._cache[key]

        return None

    def set(self, text: str, source_lang: str, target_lang: str,
            data: Dict[str, Any], ttl: Optional[int] = None) -> None:
        """
        Store translation result in cache
        """
        key = self._generate_key(text, source_lang, target_lang)
        ttl = ttl or self.default_ttl

        self._cache[key] = {
            'data': data,
            'expires_at': time.time() + ttl
        }

    def clear_expired(self) -> int:
        """
        Remove all expired items from cache and return count of removed items
        """
        current_time = time.time()
        expired_keys = [
            key for key, value in self._cache.items()
            if current_time >= value['expires_at']
        ]

        for key in expired_keys:
            del self._cache[key]

        return len(expired_keys)

    def clear_all(self) -> None:
        """
        Clear all items from cache
        """
        self._cache.clear()

# Global cache instance
translation_cache = SimpleCache()