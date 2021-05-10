'''
???
'''


class GraderInterface:
    pass

'''
def _submit_job_cb(self, request):
    id = '{}-py-{}'.format('plan_tracer_job',uuid.uuid1().hex)

    self._jobs.append({
        'type': request.type,
        'params': json.loads(request.params),
        'id': id
    })

    response = JobResponse()
    response.job_id = id
    return response

def _pending_jobs_cb(self, request):
    response = PendingJobsResponse()

    for j in self._jobs:
        response.job_ids.append(j['job_id'])
        response.types.append(j['type'])
        response.params.append(json.dumps(j['params']))

    return response
'''
