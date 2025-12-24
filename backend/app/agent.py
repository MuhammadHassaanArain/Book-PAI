import os
import cohere
from dotenv import load_dotenv
from pydantic import BaseModel
from qdrant_client import QdrantClient
from fastapi import APIRouter, HTTPException
from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI, RunConfig, set_tracing_disabled, function_tool

load_dotenv()
set_tracing_disabled(disabled=True)

router = APIRouter()


gemini_api_key = os.getenv("GEMINI_API_KEY_CHATBOT")
qdrant_url = os.getenv("QDRANT_URL")
qdrant_key = os.getenv("QDRANT_KEY")

provider = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=provider
)

config = RunConfig(
    model=model,
    model_provider=provider,
    tracing_disabled=True
)

cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_key
)

def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]

@function_tool
def retrieve(query):
    embedding = get_embedding(query)
    result = qdrant.query_points(
        collection_name="book_content",
        query=embedding,
        limit=5
    )
    return [point.payload["text"] for point in result.points]


agent = Agent(
    name="Assistant",
    instructions="""
You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
To answer the user question, first call the tool `retrieve` with the user query.
Use ONLY the returned content from `retrieve` to answer.
If the answer is not in the retrieved content, say "I don't know".
""",
    model=model,
    tools=[retrieve]
)

class ChatRequest(BaseModel):
    message: str

class ChatResponse(BaseModel):
    response: str
    status: str

@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    try:
        if not request.message:
            raise HTTPException(status_code=400, detail="Message is required")

        result = await Runner.run(agent, request.message, run_config=config)

        return ChatResponse(
            response=result.final_output,
            status="success"
        )

    except Exception as e:
        print(f"Error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))
