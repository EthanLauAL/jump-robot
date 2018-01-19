FROM ubuntu

RUN apt-get update
RUN apt-get install -y libopencv-dev

WORKDIR /app
COPY pos ./pos
COPY neg ./neg
COPY train ./
RUN mkdir xml

CMD ./train
