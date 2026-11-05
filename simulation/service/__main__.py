import os
import uvicorn

def main():
    uvicorn.run("simulation.service.app:app",host="127.0.0.1",port=int(os.getenv("ROBOTRACERS_HTTP_PORT","8000")),log_level="info")

if __name__=="__main__":main()
