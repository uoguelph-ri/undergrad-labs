FROM python:2

RUN pip install --upgrade pip && \
    pip install \
    ipython==1.2.1 \
    opencv-python \
    tornado \
    jinja2 \
    pyzmq