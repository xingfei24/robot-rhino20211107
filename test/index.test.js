import { describe, it } from 'node:test';
import assert from 'node:assert';
import { createServer } from 'node:http';

describe('HTTP Server', () => {
  it('should return 200 status code', async () => {
    const server = createServer((req, res) => {
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ message: 'Hello from Node.js!' }));
    });

    await new Promise(resolve => server.listen(0, resolve));
    const port = server.address().port;

    const response = await fetch(`http://localhost:${port}`);
    assert.strictEqual(response.status, 200);

    server.close();
  });
});