FROM nginx:latest
WORKDIR /usr/share/nginx/html
RUN rm -rf ./*
COPY dist/assets ./assets/
COPY dist/index.html .
COPY dist/vite.svg .
ENTRYPOINT [ "nginx", "-g", "daemon off;" ]