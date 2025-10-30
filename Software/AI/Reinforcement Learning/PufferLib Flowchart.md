## python -m pufferlib.pufferl train puffer_envname
- pufferl.py train()
  - pufferl.py train() before vecenv
  - vecenv = None
    - pufferl.py load_env() begin
      - pufferl.py load_env() about to pufferlib.vector.make(make_env
      - vector.py make() begin num_envs = 2
        - vector.py make() if not isinstance(env_creator_or_creators, (list, tuple)):
        - vector.py make() env_creators = [<class 'pufferlib.ocean.env.env.Env'>, <class 'pufferlib.ocean.env.env.Env'>]
        - vector.py make() before bkend = backend(env_creators, env_args, env_kwargs, num_envs, **kwargs)
        - vector.py make() env_args = [ [], [] ]
        - vector.py make() env_kwargs = [{'num_envs': 4096, 'num_agents': 1, 'num_cpu': 10, 'illegal_move_penalty': 0, 'continuous': 0}, {'num_envs': 4096, 'num_agents': 1, 'num_cpu': 10, 'illegal_move_penalty': 0, 'continuous': 0}]
        - vector.py make() num_envs = 2
          - env.py __init__() begin
            - env.py before binding.shared()
            - env.py my_shared() in binding.c begin
              - binding.c my_shared() loop through shared data
              - binding.c my_shared about to return PyLong_FromVoidPtr(state)
            - env.py my_shared() in binding.c end
            - env.py after binding.shared()
            - env.py before for i in range(num_envs):
            - env.py after for i in range(num_envs):
            - env.py before self.c_envs = binding.vectorize(*c_envs)
            - env.py after self.c_envs = binding.vectorize(*c_envs)
          - env.py __init__() end
      - make vector.py bkend = <pufferlib.vector.Multiprocessing object at 0x7a0506db3a10>
      - make vector.py end
    - thing = <pufferlib.vector.Multiprocessing object at 0x7a0506db3a10>
    - pufferl.py load_env() end
  - pufferl.py train() after vecenv
```py
    train_config = dict(**args['train'], env=env_name)
    pufferl = PuffeRL(train_config, vecenv, policy, logger) # Creates PuffeRL class

    all_logs = []
    while pufferl.global_step < train_config['total_timesteps']:
        pufferl.evaluate()
        logs = pufferl.train() # PuffeRL's method train()

        if logs is not None:
            if pufferl.global_step > 0.20*train_config['total_timesteps']:
                all_logs.append(logs)
```
```py
    @record
    def train(self):
        profile = self.profile
        epoch = self.epoch
        profile('train', epoch)
        losses = defaultdict(float)
        config = self.config
        device = config['device']

        b0 = config['prio_beta0']
        a = config['prio_alpha']
        clip_coef = config['clip_coef']
        vf_clip = config['vf_clip_coef']
        anneal_beta = b0 + (1 - b0)*a*self.epoch/self.total_epochs
        self.ratio[:] = 1

        for mb in range(self.total_minibatches):
            profile('train_misc', epoch, nest=True)
            self.amp_context.__enter__()

            shape = self.values.shape
            advantages = torch.zeros(shape, device=device)
            advantages = compute_puff_advantage(self.values, self.rewards,
                self.terminals, self.ratio, advantages, config['gamma'],
                config['gae_lambda'], config['vtrace_rho_clip'], config['vtrace_c_clip'])

            profile('train_copy', epoch)
            adv = advantages.abs().sum(axis=1)
            prio_weights = torch.nan_to_num(adv**a, 0, 0, 0)
            prio_probs = (prio_weights + 1e-6)/(prio_weights.sum() + 1e-6)
            idx = torch.multinomial(prio_probs, self.minibatch_segments)
            mb_prio = (self.segments*prio_probs[idx, None])**-anneal_beta
            mb_obs = self.observations[idx]
            mb_actions = self.actions[idx]
            mb_logprobs = self.logprobs[idx]
            mb_rewards = self.rewards[idx]
            mb_terminals = self.terminals[idx]
            mb_truncations = self.truncations[idx]
            mb_ratio = self.ratio[idx]
            mb_values = self.values[idx]
            mb_returns = advantages[idx] + mb_values
            mb_advantages = advantages[idx]

            profile('train_forward', epoch)
            if not config['use_rnn']:
                mb_obs = mb_obs.reshape(-1, *self.vecenv.single_observation_space.shape)

            state = dict(
                action=mb_actions,
                lstm_h=None,
                lstm_c=None,
            )

            logits, newvalue = self.policy(mb_obs, state)
            actions, newlogprob, entropy = pufferlib.pytorch.sample_logits(logits, action=mb_actions)

            profile('train_misc', epoch)
            newlogprob = newlogprob.reshape(mb_logprobs.shape)
            logratio = newlogprob - mb_logprobs
            ratio = logratio.exp()
            self.ratio[idx] = ratio.detach()

            with torch.no_grad():
                old_approx_kl = (-logratio).mean()
                approx_kl = ((ratio - 1) - logratio).mean()
                clipfrac = ((ratio - 1.0).abs() > config['clip_coef']).float().mean()

            adv = advantages[idx]
            adv = compute_puff_advantage(mb_values, mb_rewards, mb_terminals,
                ratio, adv, config['gamma'], config['gae_lambda'],
                config['vtrace_rho_clip'], config['vtrace_c_clip'])
            adv = mb_advantages
            adv = mb_prio * (adv - adv.mean()) / (adv.std() + 1e-8)

            # Losses
            pg_loss1 = -adv * ratio
            pg_loss2 = -adv * torch.clamp(ratio, 1 - clip_coef, 1 + clip_coef)
            pg_loss = torch.max(pg_loss1, pg_loss2).mean()

            newvalue = newvalue.view(mb_returns.shape)
            v_clipped = mb_values + torch.clamp(newvalue - mb_values, -vf_clip, vf_clip)
            v_loss_unclipped = (newvalue - mb_returns) ** 2
            v_loss_clipped = (v_clipped - mb_returns) ** 2
            v_loss = 0.5*torch.max(v_loss_unclipped, v_loss_clipped).mean()

            entropy_loss = entropy.mean()

            loss = pg_loss + config['vf_coef']*v_loss - config['ent_coef']*entropy_loss
            self.amp_context.__enter__() # TODO: AMP needs some debugging

            # This breaks vloss clipping?
            self.values[idx] = newvalue.detach().float()

            # Logging
            profile('train_misc', epoch)
            losses['policy_loss'] += pg_loss.item() / self.total_minibatches
            losses['value_loss'] += v_loss.item() / self.total_minibatches
            losses['entropy'] += entropy_loss.item() / self.total_minibatches
            losses['old_approx_kl'] += old_approx_kl.item() / self.total_minibatches
            losses['approx_kl'] += approx_kl.item() / self.total_minibatches
            losses['clipfrac'] += clipfrac.item() / self.total_minibatches
            losses['importance'] += ratio.mean().item() / self.total_minibatches

            # Learn on accumulated minibatches
            profile('learn', epoch)
            loss.backward()
            if (mb + 1) % self.accumulate_minibatches == 0:
                torch.nn.utils.clip_grad_norm_(self.policy.parameters(), config['max_grad_norm'])
                self.optimizer.step()
                self.optimizer.zero_grad()

        # Reprioritize experience
        profile('train_misc', epoch)
        if config['anneal_lr']:
            self.scheduler.step()

        y_pred = self.values.flatten()
        y_true = advantages.flatten() + self.values.flatten()
        var_y = y_true.var()
        explained_var = torch.nan if var_y == 0 else 1 - (y_true - y_pred).var() / var_y
        losses['explained_variance'] = explained_var.item()

        profile.end()
        logs = None
        self.epoch += 1
        done_training = self.global_step >= config['total_timesteps']
        if done_training or self.global_step == 0 or time.time() > self.last_log_time + 0.25:
            logs = self.mean_and_log()
            self.losses = losses
            self.print_dashboard()
            self.stats = defaultdict(list)
            self.last_log_time = time.time()
            self.last_log_step = self.global_step
            profile.clear()

        if self.epoch % config['checkpoint_interval'] == 0 or done_training:
            self.save_checkpoint()
            self.msg = f'Checkpoint saved at update {self.epoch}'

        return logs
```

compute_puff_advantage is in:
- pufferlib/extensions/pufferlib.cpp
- or
- pufferlib/extensions/cuda/pufferlib.cu

[[PufferLib]]
