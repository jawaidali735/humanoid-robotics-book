"""
Hugging Face Space application entry point.
This file creates the FastAPI app instance for deployment on Hugging Face Spaces.
"""
import os
import sys
from api_integration.main import app

# For Hugging Face Spaces, make sure the app is available at the top level
# The gradio interface or direct FastAPI access will use this app instance
if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8007))  # Hugging Face provides PORT environment variable
    uvicorn.run(app, host="0.0.0.0", port=port)