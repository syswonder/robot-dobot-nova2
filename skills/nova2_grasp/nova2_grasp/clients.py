"""Consume arm, gripper and SDK camera MCP capabilities through Atlas channels."""
from __future__ import annotations

import asyncio
import json
from datetime import timedelta

from robonix_api import ATLAS
from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client


class Primitives:
    def __init__(self, provider, config):
        self.provider, self.config = provider, config

    async def _call(self, group, name, args):
        prefix = {"arm":"robonix/primitive/dobot_nova2/", "gripper":"robonix/primitive/tg9801/",
                  "camera":"robonix/primitive/orbbec_camera/"}[group]
        ident = self.config[group+"_provider_id"]
        capability = ATLAS.find_unique_capability(contract_id=prefix+name, provider_id=ident, transport="mcp")
        edge = self.provider.connect_capability(capability, prefix+name, "mcp")
        try:
            async with streamablehttp_client(edge.endpoint, timeout=timedelta(seconds=15),
                                            sse_read_timeout=timedelta(seconds=25)) as (read, write, _):
                async with ClientSession(read, write, read_timeout_seconds=timedelta(seconds=25)) as session:
                    await session.initialize()
                    result = await session.call_tool(name, args)
                    if result.isError:
                        raise RuntimeError(f"{group}/{name}: MCP tool error")
                    response = result.structuredContent
                    if response is None:
                        response = json.loads("".join(part.text for part in result.content if hasattr(part, "text")))
                    if not response.get("ok"):
                        raise RuntimeError(f"{group}/{name}: {response.get('message', 'failed')}")
                    return json.loads(response["result_json"])
        finally:
            edge.close()

    def call(self, group, name, **args):
        return asyncio.run(self._call(group, name, args))
