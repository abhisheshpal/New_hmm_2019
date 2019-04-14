# Thorvald Operations Centre (TOC)

## Server setup

### Crontab (user rasberry)

```
@reboot /usr/local/bin/ngrok1 -config /home/rasberry/.ngrok -log-level WARNING -log stdout start-all | /usr/bin/logger -t ngrok
*/5 * * * * /home/rasberry/network-scripts/iptracker.sh -n RASberry-server
```

### `.ngrok`

```
server_addr: ngrok.lcas.group:4443
trust_host_root_certs: true
auth_token: 6hsy4H+U8G6+I1iNMzMXYHVNZ9TGs+H87dta91d0BxY=
tunnels:
  toc:
    subdomain: toc
    proto:
      https: 5889
  toc_slack:
    subdomain: toc_slack
    proto:
      https: 8080
  toc_shell:
    subdomain: toc_shell
    proto:
      https: 4200
```
