# Dockerfile Cheatsheet

This cheatsheet provides a quick reference for writing **[[Dockerfile]]s** to build container images.  
It focuses on **common one-liners** and their purpose.

---

## Core Instructions

`FROM ubuntu:20.04` → Use base image  
`FROM python:3.10-slim` → Use specific Python version image  
`MAINTAINER "Your Name <you@example.com>"` → Set maintainer (deprecated, use LABEL instead)  
`LABEL version="1.0"` → Add metadata label  
`WORKDIR /app` → Set working directory  
`COPY . /app` → Copy local files into container  
`ADD file.tar.gz /app/` → Copy and extract archive into container  
`RUN apt-get update && apt-get install -y curl` → Run shell commands  
`RUN pip install -r requirements.txt` → Install Python dependencies  
`ENV APP_ENV=production` → Set environment variable  
`ARG VERSION=latest` → Define build-time argument  
`EXPOSE 8080` → Inform container runtime about open port  
`USER appuser` → Switch user inside container  
`CMD ["python", "app.py"]` → Default command to run  
`ENTRYPOINT ["python", "app.py"]` → Always run this command (args appended)  

---

## Common Patterns

`RUN groupadd -r app && useradd -r -g app app` → Add non-root user  
`COPY requirements.txt ./` → Copy requirements first for caching  
`RUN pip install --no-cache-dir -r requirements.txt` → Install deps efficiently  
`CMD ["./start.sh"]` → Run startup script  
`ENTRYPOINT ["docker-entrypoint.sh"]` → Define entrypoint script  

---

## Multi-stage Builds

`FROM golang:1.18 as builder` → First stage build image  
`WORKDIR /src` → Set build context  
`COPY . .` → Copy source  
`RUN go build -o myapp` → Build binary  
`FROM alpine:3.17` → Minimal runtime image  
`COPY --from=builder /src/myapp /usr/local/bin/myapp` → Copy artifact  
`CMD ["myapp"]` → Run final binary  

---

## Best Practices

- **Use slim/minimal base images** to reduce size (`alpine`, `debian-slim`, etc).  
- **Order instructions logically**: install dependencies → copy source → build → run.  
- **Use multi-stage builds** to separate build-time dependencies.  
- **Use ENTRYPOINT for executables**, CMD for default arguments.  
- **Pin versions** for reproducibility (`apt-get install -y curl=7.68.0-1ubuntu2.18`).  

---
