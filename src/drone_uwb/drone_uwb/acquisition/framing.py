"""Bounded UART byte framing without corrections."""
class LineFramer:
    """Bound storage and discard an oversized line through its next newline."""

    def __init__(self, limit=8192):
        self.limit = limit
        self.buffer = bytearray()
        self.discarding = False
        self.overflows = 0

    def feed(self, chunk):
        lines = []
        for piece_index, piece in enumerate(chunk.split(b'\n')):
            if piece_index:
                if not self.discarding and self.buffer:
                    lines.append(bytes(self.buffer))
                self.buffer.clear()
                self.discarding = False
            if not self.discarding:
                if len(self.buffer) + len(piece) > self.limit:
                    self.buffer.clear()
                    self.discarding = True
                    self.overflows += 1
                else:
                    self.buffer.extend(piece)
        return lines
