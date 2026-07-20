# Onshape CFD Workflow

Onshape is not local CLI-first, but it can still be driven from agents via REST.

## 1) Expected architecture

You need:
- One authenticated service account/service user with API key or OAuth.
- A repository document that stores a template feature model.
- A separate loop runner that reads parameter rows and calls Onshape endpoints.

## 2) Param-driven approach

Store parameters in `json` or `xlsx` and map them to feature variables in a known feature stack or template document:

1. Create/maintain an Onshape Part Studio template.
2. Expose parameters such as `diffuser_angle`, `inlet_radius`, `outlet_radius`.
3. Call Onshape endpoint to create/update feature values.
4. Export resulting parts as STEP.

## 3) Loop skeleton (pseudo)

```python
import json
import requests

auth = {"Authorization": "Bearer ..."}  # based on Onshape API auth choice
base = "https://cad.onshape.com/api/v6"

def derive_from_params(params):
    # map to document/revision and feature ids
    payload = {
        "featureId": "uuid-here",
        "inputParams": [
            {"id": "diffuser_angle", "value": params["diffuser_angle_deg"]},
            {"id": "inlet_radius", "value": params["inlet_radius_m"]},
            {"id": "outlet_radius", "value": params["outlet_radius_m"]},
        ],
    }
    return payload

run = json.load(open("params/runs/run_001.json"))
payload = derive_from_params(run)
r = requests.post(f"{base}/some-endpoint", headers=auth, json=payload, timeout=30)
print(r.status_code, r.text)
```

Replace endpoint and IDs with those from your exact Onshape API workflow.

## 4) Practical cautions

- API usage requires stable key custody and quota handling.
- You should not hardcode credentials in agent scripts.
- Build a retry policy for HTTP 429 / 5xx.
- Cache docs + feature IDs for each template to avoid accidental edits.

## 5) Why this is still useful for CFD optimization

- Good governance and revisioning.
- Excellent for teams already on Onshape.
- Not ideal if all loop steps must be fully local and offline.

## 6) References

- API intro and examples: https://onshape-public.github.io/docs/api-intro/
- API keys and auth model: https://onshape-public.github.io/docs/auth/apikeys/
- Pricing and plan expectations: https://www.onshape.com/en/pricing
