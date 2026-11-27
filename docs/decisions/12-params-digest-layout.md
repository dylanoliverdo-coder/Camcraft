# Split the approach digest writer

Move the approach fields from the link trailer into a private module. They
share a single opt-in tag and form a complete block. Preserve the order and
bit representation of every hash write, including the default values that
write no bytes. The existing parameter module line-count test covers the
new file as well as the original.
