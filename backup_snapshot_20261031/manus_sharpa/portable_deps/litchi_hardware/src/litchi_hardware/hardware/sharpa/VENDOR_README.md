# Bundled Sharpa SDK assets

`tools/vendor_sdks.py` stages the proprietary Sharpa Wave SDK assets into
`_vendor/` for the organization-internal Linux x86_64 wheel.

The assets originate from
`reference/sharpa_extracted/opt/sharpa-wave-sdk` and include SDK 5.0.6 headers,
libraries, configuration, `pilot_sdk`, and supported CPython extensions.

Do not publish these assets to a public package index. Distribution is limited
to recipients authorized by the applicable vendor agreement.
