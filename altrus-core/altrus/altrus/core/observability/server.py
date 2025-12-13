from prometheus_client import start_http_server
from altrus.core.observability.metrics import REGISTRY
from prometheus_client.exposition import make_wsgi_app
from wsgiref.simple_server import make_server
import threading


def start_metrics_server(port=8000):
    app = make_wsgi_app(REGISTRY)

    def serve():
        httpd = make_server("", port, app)
        httpd.serve_forever()

    t = threading.Thread(target=serve, daemon=True)
    t.start()
