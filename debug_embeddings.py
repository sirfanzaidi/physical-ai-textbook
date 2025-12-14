#!/usr/bin/env python3
"""Debug script to check Cohere embedding response format."""

import os
import sys
from dotenv import load_dotenv
import cohere
import json

# Fix encoding on Windows
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

load_dotenv()

api_key = os.getenv('COHERE_API_KEY')
if not api_key:
    print("No COHERE_API_KEY found in .env")
    exit(1)

try:
    # Initialize client
    client = cohere.ClientV2(api_key=api_key)
    print("[OK] Cohere client initialized")

    # Test embedding
    response = client.embed(
        texts=["Hello world", "Test embedding"],
        model="embed-v4.0",
        input_type="search_document"
    )

    print("\n=== RESPONSE ANALYSIS ===")
    print(f"Response type: {type(response)}")
    print(f"Response class: {response.__class__.__name__}")

    # Check embeddings attribute
    if hasattr(response, 'embeddings'):
        emb = response.embeddings
        print(f"\n[OK] Has 'embeddings' attribute")
        print(f"   Type: {type(emb)}")
        print(f"   Class: {emb.__class__.__name__}")

        # If it's an object, check its attributes
        if not isinstance(emb, list):
            print(f"   Is an OBJECT, checking attributes...")

            # Try accessing as list
            try:
                print(f"   Can convert to list: {type(list(emb))}")
                converted = list(emb)
                if converted:
                    print(f"   Converted list length: {len(converted)}")
                    print(f"   First item type: {type(converted[0])}")
                    if isinstance(converted[0], (list, tuple)):
                        print(f"   First item length: {len(converted[0])}")
                        print(f"   First item (first 3 values): {converted[0][:3]}")
            except Exception as e:
                print(f"   Cannot convert to list: {e}")

            # Check data attribute
            if hasattr(emb, 'data'):
                print(f"\n   Has 'data' attribute")
                data = emb.data
                print(f"      Type: {type(data)}")
                if isinstance(data, list) and len(data) > 0:
                    print(f"      Is a list with {len(data)} items")
                    print(f"      First item type: {type(data[0])}")
                    if isinstance(data[0], (list, tuple)):
                        print(f"      First item length: {len(data[0])}")
                        print(f"      First item (first 3 values): {data[0][:3]}")

            # Try iterating
            print(f"\n   Trying to iterate...")
            count = 0
            for i, item in enumerate(emb):
                if i == 0:
                    print(f"   First iteration item type: {type(item)}")
                    if isinstance(item, (list, tuple)):
                        print(f"   First iteration item length: {len(item)}")
                        print(f"   First iteration item (first 3): {item[:3]}")
                count += 1
                if count >= 2:
                    break
        else:
            print(f"   Is a list with {len(emb)} items")
            if emb and len(emb) > 0:
                print(f"   First embedding type: {type(emb[0])}")
                if isinstance(emb[0], (list, tuple)):
                    print(f"   First embedding length: {len(emb[0])}")
                    print(f"   First embedding (first 5 dims): {emb[0][:5]}")

    print("\n[DONE]")

except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
