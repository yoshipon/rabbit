class AlsaAudio
{
  snd_pcm_t *handle;
  snd_pcm_hw_params_t *params;

  int error(int err)
  {
    // TODO: 例外処理
    if (err < 0) {
      string str_error = snd_strerror(err);
      ROS_ERROR_STREAM("[ASLA ERROR]: " << str_error << " (Code: " << err << ")");
    }

    assert(err >= 0);

    return err;
  }

public:
  AlsaAudio() {}

  void open(string dev_name, snd_pcm_stream_t mode)
  {
    error(snd_pcm_open(&handle, dev_name.c_str(), mode, 0));
    error(snd_pcm_hw_params_malloc(&params));
    error(snd_pcm_hw_params_any(handle, params));
  }

  template<typename Op, typename Arg1>
  int set(Op op, Arg1 arg1) { return error(op(handle, params, arg1)); }

  template<typename Op, typename Arg1, typename Arg2>
  int set(Op op, Arg1 arg1, Arg2 arg2) { return error(op(handle, params, arg1, arg2)); }

  void prepare()
  {
    error(snd_pcm_hw_params(handle, params));
    error(snd_pcm_prepare(handle));
  }
 
  template<typename Any>
  int read(const DenseBase<Any> &buf, snd_pcm_uframes_t nbuf)
  { 
    return error(snd_pcm_readi(handle, const_cast<DenseBase<Any>&>(buf).derived().data(), nbuf));
  }

  template<typename Any>
  int write(const DenseBase<Any> &buf, snd_pcm_uframes_t nbuf)
  {
    return error(snd_pcm_writei(handle, buf.derived().data(), nbuf));
  }

  void close()
  {
    if (!handle) {
      snd_pcm_hw_params_free(params);
      snd_pcm_close(handle);
    }
  }

  ~AlsaAudio() {
    close();
  }
};
