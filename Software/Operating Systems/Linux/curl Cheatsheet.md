# curl Cheatsheet

curl is a command-line tool for transferring data using various network protocols. It's one of the most versatile tools for testing APIs, downloading files, debugging network requests, and automating HTTP interactions. This cheatsheet covers comprehensive curl usage for developers, DevOps engineers, and system administrators.

## 🎯 Overview

---

curl (Client URL) supports numerous protocols including HTTP, HTTPS, FTP, SFTP, SMTP, and more. It's pre-installed on most Unix-like systems and Windows 10+, making it a universal tool for network operations. The tool is scriptable, supports advanced features like authentication, cookies, and custom headers, and provides detailed debugging information.

## 🚀 Basic Usage

---

**Simple GET Request**: `curl https://example.com`

**Save Output to File**: `curl -o filename.html https://example.com` or `curl -O https://example.com/file.zip` (uses remote filename)

**Follow Redirects**: `curl -L https://example.com`

**Show Response Headers**: `curl -i https://example.com` (includes headers) or `curl -I https://example.com` (headers only)

**Silent Mode**: `curl -s https://example.com` (suppress progress bar)

**Verbose Output**: `curl -v https://example.com` (shows detailed connection info)

**Download with Progress Bar**: `curl -# -O https://example.com/largefile.zip`

## 📡 HTTP Methods

---

**GET (default)**: `curl https://api.example.com/users`

**POST with Data**: `curl -X POST https://api.example.com/users -d "name=John&age=30"`

**POST JSON Data**: `curl -X POST https://api.example.com/users -H "Content-Type: application/json" -d '{"name":"John","age":30}'`

**POST from File**: `curl -X POST https://api.example.com/users -d @data.json -H "Content-Type: application/json"`

**PUT Request**: `curl -X PUT https://api.example.com/users/123 -d '{"name":"Jane"}' -H "Content-Type: application/json"`

**PATCH Request**: `curl -X PATCH https://api.example.com/users/123 -d '{"age":31}' -H "Content-Type: application/json"`

**DELETE Request**: `curl -X DELETE https://api.example.com/users/123`

**HEAD Request**: `curl -I https://api.example.com/users` or `curl --head https://api.example.com/users`

**OPTIONS Request**: `curl -X OPTIONS https://api.example.com/users -i`

## 🔑 Authentication

---

**Basic Auth**: `curl -u username:password https://api.example.com/protected`

**Basic Auth (prompt for password)**: `curl -u username https://api.example.com/protected`

**Bearer Token**: `curl -H "Authorization: Bearer YOUR_TOKEN_HERE" https://api.example.com/protected`

**API Key in Header**: `curl -H "X-API-Key: YOUR_API_KEY" https://api.example.com/data`

**Digest Auth**: `curl --digest -u username:password https://api.example.com/protected`

**OAuth2 Token**: `curl -H "Authorization: OAuth YOUR_OAUTH_TOKEN" https://api.example.com/data`

**Certificate-Based Auth**: `curl --cert client.pem --key key.pem https://api.example.com/secure`

**Negotiate/Kerberos**: `curl --negotiate -u : https://api.example.com/protected`

## 📋 Headers

---

**Custom Header**: `curl -H "X-Custom-Header: value" https://api.example.com`

**Multiple Headers**: `curl -H "Content-Type: application/json" -H "Accept: application/json" -H "Authorization: Bearer token" https://api.example.com`

**User Agent**: `curl -A "Mozilla/5.0" https://example.com` or `curl -H "User-Agent: CustomBot/1.0" https://example.com`

**Referer Header**: `curl -e "https://google.com" https://example.com` or `curl -H "Referer: https://google.com" https://example.com`

**Accept Header**: `curl -H "Accept: application/json" https://api.example.com`

**Content-Type**: `curl -H "Content-Type: application/xml" https://api.example.com`

**Remove Default Header**: `curl -H "Accept:" https://example.com`

## 🍪 Cookies

---

**Send Cookie**: `curl -b "session=abc123" https://example.com`

**Multiple Cookies**: `curl -b "session=abc123; user=john" https://example.com`

**Cookie from File**: `curl -b cookies.txt https://example.com`

**Save Cookies**: `curl -c cookies.txt https://example.com/login -d "user=john&pass=secret"`

**Use and Save Cookies**: `curl -b cookies.txt -c cookies.txt https://example.com`

**Cookie Jar (session)**: `curl -b cookies.txt -c cookies.txt https://example.com/login -d "creds" && curl -b cookies.txt https://example.com/dashboard`

## 📤 File Uploads

---

**Upload File (multipart/form-data)**: `curl -F "file=@/path/to/file.txt" https://api.example.com/upload`

**Upload Multiple Files**: `curl -F "file1=@image1.jpg" -F "file2=@image2.jpg" https://api.example.com/upload`

**Upload with Form Fields**: `curl -F "file=@document.pdf" -F "title=MyDocument" -F "description=Important file" https://api.example.com/upload`

**Specify Filename**: `curl -F "file=@localfile.txt;filename=remotefile.txt" https://api.example.com/upload`

**Specify Content-Type**: `curl -F "file=@data.json;type=application/json" https://api.example.com/upload`

**Upload Raw Binary**: `curl -X POST --data-binary "@file.bin" https://api.example.com/upload`

**Upload via PUT**: `curl -T file.txt https://ftp.example.com/upload/`

**Upload from stdin**: `echo "data" | curl -X POST --data-binary @- https://api.example.com/upload`

## 📥 Downloads

---

**Download File**: `curl -O https://example.com/file.zip`

**Download with Custom Name**: `curl -o myfile.zip https://example.com/download`

**Resume Download**: `curl -C - -O https://example.com/largefile.iso`

**Download Multiple Files**: `curl -O https://example.com/file1.txt -O https://example.com/file2.txt`

**Download with Rate Limit**: `curl --limit-rate 100K -O https://example.com/file.zip`

**Download with Timeout**: `curl -m 300 -O https://example.com/file.zip` (300 seconds max)

**Connection Timeout**: `curl --connect-timeout 10 https://example.com`

**Retry on Failure**: `curl --retry 5 --retry-delay 3 -O https://example.com/file.zip`

**Download FTP File**: `curl -u username:password ftp://ftp.example.com/file.txt -O`

## 🔍 Query Parameters

---

**Single Parameter**: `curl "https://api.example.com/search?q=test"`

**Multiple Parameters**: `curl "https://api.example.com/search?q=test&limit=10&offset=0"`

**URL Encoding**: `curl --data-urlencode "name=John Doe" https://api.example.com/users` or `curl -G --data-urlencode "q=hello world" https://api.example.com/search`

**Complex Query**: `curl "https://api.example.com/products?category=electronics&price[min]=100&price[max]=500&sort=price"`

## 🔒 HTTPS/SSL/TLS

---

**Ignore SSL Certificate**: `curl -k https://self-signed.example.com` or `curl --insecure https://self-signed.example.com`

**Specify CA Certificate**: `curl --cacert ca-bundle.crt https://example.com`

**Client Certificate**: `curl --cert client.pem --key key.pem https://example.com`

**Show SSL Certificate**: `curl -v https://example.com 2>&1 | grep -A 10 "Server certificate"`

**TLS Version**: `curl --tlsv1.2 https://example.com` or `curl --tls-max 1.2 https://example.com`

**Cipher Suite**: `curl --ciphers ECDHE-RSA-AES256-GCM-SHA384 https://example.com`

## 🐛 Debugging and Testing

---

**Verbose Output**: `curl -v https://api.example.com` (shows request/response headers and SSL handshake)

**Trace ASCII**: `curl --trace-ascii debug.txt https://api.example.com` (logs everything to file)

**Trace Binary**: `curl --trace trace.bin https://api.example.com`

**Timing Information**: `curl -w "@curl-format.txt" -o /dev/null -s https://example.com` where curl-format.txt contains: `time_namelookup: %{time_namelookup}\ntime_connect: %{time_connect}\ntime_appconnect: %{time_appconnect}\ntime_pretransfer: %{time_pretransfer}\ntime_redirect: %{time_redirect}\ntime_starttransfer: %{time_starttransfer}\ntime_total: %{time_total}\n`

**Show Only HTTP Code**: `curl -s -o /dev/null -w "%{http_code}" https://example.com`

**Show Response Time**: `curl -w "Time: %{time_total}s\n" -o /dev/null -s https://example.com`

**DNS Resolution**: `curl --dns-servers 8.8.8.8 https://example.com`

**Resolve Custom Host**: `curl --resolve example.com:443:127.0.0.1 https://example.com`

**IPv4 Only**: `curl -4 https://example.com`

**IPv6 Only**: `curl -6 https://example.com`

## 📊 Output Formatting

---

**Format Output**: `curl -w "\nHTTP: %{http_code}\nSize: %{size_download}\n" https://example.com`

**JSON Pretty Print**: `curl https://api.example.com/data | jq .` (requires jq)

**Silent with Status**: `curl -s -o response.json -w "%{http_code}" https://api.example.com`

**Write to File**: `curl https://api.example.com/data -o output.json`

**Append to File**: `curl https://api.example.com/data >> log.txt`

**Null Output**: `curl -o /dev/null https://example.com` (discard response body)

## 🔄 Redirects and Location

---

**Follow Redirects**: `curl -L https://example.com`

**Max Redirects**: `curl -L --max-redirs 5 https://example.com`

**Show Redirect Chain**: `curl -sL -w "URL: %{url_effective}\nCode: %{http_code}\n" https://example.com`

**Don't Follow Redirects**: `curl https://example.com` (default behavior)

## 🌐 Proxy

---

**HTTP Proxy**: `curl -x http://proxy.example.com:8080 https://api.example.com`

**Proxy with Auth**: `curl -x http://user:pass@proxy.example.com:8080 https://api.example.com`

**SOCKS Proxy**: `curl --socks5 localhost:1080 https://api.example.com`

**SOCKS5 with DNS**: `curl --socks5-hostname localhost:1080 https://api.example.com`

**No Proxy for Localhost**: `curl --noproxy localhost,127.0.0.1 -x proxy.example.com:8080 https://api.example.com`

**Environment Variable**: Set `export http_proxy=http://proxy.example.com:8080` then `curl https://example.com`

## 📨 Form Data

---

**URL-Encoded Form**: `curl -d "username=john&password=secret" https://example.com/login`

**Form from File**: `curl -d @formdata.txt https://example.com/submit`

**Multiple Data Parameters**: `curl -d "name=John" -d "age=30" -d "city=NYC" https://api.example.com/users`

**GET with Data**: `curl -G -d "q=search" -d "limit=10" https://api.example.com/search`

**Multipart Form**: `curl -F "name=John" -F "age=30" https://api.example.com/users`

**Empty POST**: `curl -X POST https://api.example.com/trigger`

## 🔢 HTTP/2 and HTTP/3

---

**Force HTTP/2**: `curl --http2 https://example.com`

**HTTP/2 Prior Knowledge**: `curl --http2-prior-knowledge http://example.com`

**Force HTTP/1.1**: `curl --http1.1 https://example.com`

**HTTP/3**: `curl --http3 https://example.com` (requires curl built with HTTP/3 support)

**Show Protocol**: `curl -w "Protocol: %{http_version}\n" -o /dev/null -s https://example.com`

## 📋 Comparison Chart

---

| Aspect | curl | wget | HTTPie | Postman | Insomnia |
|--------|------|------|--------|---------|----------|
| **Type** | CLI tool | CLI tool | CLI tool | GUI application | GUI application |
| **Best For** | Scripting, APIs | File downloads | Human-friendly API testing | Interactive API development | REST/GraphQL APIs |
| **Protocols** | 20+ protocols | HTTP, HTTPS, FTP | HTTP only | HTTP/HTTPS | HTTP/HTTPS, GraphQL |
| **Learning Curve** | Moderate | Easy | Very easy | Easy | Easy |
| **JSON Support** | Manual | Manual | Built-in formatting | Built-in | Built-in |
| **Scripting** | Excellent | Good | Good | Limited (Newman) | Limited |
| **Default Behavior** | Output to stdout | Download files | Pretty output | Interactive GUI | Interactive GUI |
| **Authentication** | All types | Basic/Digest | Sessions, tokens | All types | All types |
| **Platform** | Cross-platform | Cross-platform | Cross-platform | Cross-platform | Cross-platform |

## 🎨 Advanced Techniques

---

**Send JSON with Variables**: `curl -X POST https://api.example.com/users -H "Content-Type: application/json" -d "{\"name\":\"$USERNAME\",\"email\":\"$EMAIL\"}"`

**Loop Through IDs**: `for i in {1..10}; do curl https://api.example.com/users/$i; done`

**Parallel Requests**: `curl -Z https://site1.com https://site2.com https://site3.com`

**Use Config File**: Create `.curlrc` with default options like `-L`, `-v`, etc., or use `curl -K config.txt https://example.com`

**Send Binary Data**: `curl -X POST --data-binary "@image.png" -H "Content-Type: image/png" https://api.example.com/upload`

**Expand URL Pattern**: `curl https://example.com/file[1-10].txt` (downloads file1.txt through file10.txt)

**Alphabetic Range**: `curl https://example.com/section[a-z].html`

**Custom DNS**: `curl --dns-interface eth0 https://example.com`

**Unix Socket**: `curl --unix-socket /var/run/docker.sock http://localhost/containers/json`

**Mail with SMTP**: `curl smtp://mail.example.com --mail-from sender@example.com --mail-rcpt recipient@example.com --upload-file email.txt`

## 🧪 Testing and CI/CD

---

**Health Check Script**: `curl -f https://example.com/health || exit 1` (fails if non-2xx response)

**API Test with Exit Code**: `HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" https://api.example.com) && [ "$HTTP_CODE" -eq 200 ] || exit 1`

**Load Test (simple)**: `for i in {1..1000}; do curl -s https://api.example.com > /dev/null & done`

**Monitor Endpoint**: `watch -n 5 'curl -s https://api.example.com/status | jq .'`

**Benchmark Response Time**: `curl -w "@curl-timing.txt" -o /dev/null -s https://example.com` with timing format file

**Check All Endpoints**: `while read url; do curl -s -o /dev/null -w "$url: %{http_code}\n" "$url"; done < urls.txt`

## 🔗 Related Concepts

---

- [[HTTP]] (Hypertext Transfer Protocol)
- [[REST API]]
- [[API Testing]]
- [[Web Scraping]]
- [[CI-CD]] (Continuous Integration/Continuous Deployment)
- [[JSON]]
- [[XML]]
- [[OAuth]]
- [[SSL-TLS]] (Secure Sockets Layer/Transport Layer Security)
- [[DNS]] (Domain Name System)
- [[FTP]] (File Transfer Protocol)
- [[SMTP]] (Simple Mail Transfer Protocol)
- [[Bash Scripting]]
- [[DevOps]]
- [[Network Protocols]]
- [[Web Security]]

## 💡 Common Patterns

---

**API Testing Workflow**: `curl -X POST https://api.example.com/login -d '{"user":"john","pass":"secret"}' -c cookies.txt && curl -b cookies.txt https://api.example.com/protected/data`

**Download with Checksum Verify**: `curl -O https://example.com/file.tar.gz && curl -s https://example.com/file.tar.gz.sha256 | sha256sum -c`

**GraphQL Query**: `curl -X POST https://api.example.com/graphql -H "Content-Type: application/json" -d '{"query":"{ users { id name } }"}'`

**WebSocket Connection**: `curl --include --no-buffer --header "Connection: Upgrade" --header "Upgrade: websocket" --header "Sec-WebSocket-Key: SGVsbG8=" --header "Sec-WebSocket-Version: 13" https://example.com/ws`

**Rate Limited Requests**: `for i in {1..100}; do curl https://api.example.com/data; sleep 1; done`

**Scrape and Parse**: `curl -s https://example.com | grep -oP '(?<=<title>).*?(?=</title>)'`

**Monitor API Changes**: `curl -s https://api.example.com/data | md5sum` (compare hash over time)

## 🛠️ Troubleshooting

---

**Connection Refused**: Check if service is running, firewall rules, correct port

**SSL Certificate Errors**: Use `-k` for testing, install proper CA certificates, verify certificate chain

**Timeout Issues**: Increase with `-m` or `--connect-timeout`, check network connectivity

**DNS Resolution Fails**: Use `--dns-servers`, check `/etc/resolv.conf`, test with `dig` or `nslookup`

**Proxy Issues**: Verify proxy settings, check authentication, try direct connection

**Redirect Loops**: Use `-L` carefully, check for malformed Location headers, limit redirects with `--max-redirs`

**Large File Uploads**: Use `--expect100-timeout 3600`, increase timeouts, consider chunked encoding

**Authentication Fails**: Verify credentials, check auth method, examine headers with `-v`

## 📚 Environment Variables

---

**http_proxy**: Set HTTP proxy: `export http_proxy=http://proxy.example.com:8080`

**https_proxy**: Set HTTPS proxy: `export https_proxy=http://proxy.example.com:8080`

**no_proxy**: Bypass proxy for hosts: `export no_proxy=localhost,127.0.0.1,.example.com`

**CURL_CA_BUNDLE**: Specify CA certificate bundle: `export CURL_CA_BUNDLE=/path/to/ca-bundle.crt`

**CURL_HOME**: Set curl home directory for config: `export CURL_HOME=/custom/path`

## 🎯 Performance Tips

---

**Use HTTP/2**: `--http2` for multiplexing and header compression

**Reuse Connections**: Use `--keepalive-time` for connection reuse

**Compress Transfers**: `--compressed` to request compressed responses

**Limit Bandwidth**: `--limit-rate 1M` to prevent network saturation

**DNS Caching**: curl caches DNS for connection lifetime

**Parallel Downloads**: Use `&` for background jobs or GNU Parallel

**Connection Pooling**: Keep connections alive with proper timeout settings

## 📖 Further Reading

---

- Official curl documentation: https://curl.se/docs/
- curl manual page: `man curl`
- "Everything curl" book by Daniel Stenberg
- curl GitHub repository for source code and examples
- curl mailing list archives for advanced usage
- HTTPie documentation for alternative tool comparison
- RESTful API design principles
