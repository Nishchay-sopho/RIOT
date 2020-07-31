typedef struct {
	int (*get_next_hop_address) (ipv6_addr_t *addr, uint8_t iface);
} routing_interface_t;